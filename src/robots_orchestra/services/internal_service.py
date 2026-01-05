from typing import Any, Dict, Optional
import ampl
import asyncio
import numpy as np
import threading
from typing import Dict, Optional, List
from pydantic import BaseModel
from contextlib import asynccontextmanager
from fastapi import FastAPI, Request, HTTPException
from robots_orchestra.services.class_def import *

# 兼容目前ampl已经包含的预设的机器人的逆解
class IKSolver:
    def __init__(self, robot_type: str):
        self.robot_type = robot_type
        self.solver = None
        self.dof = None
        self.lock = threading.Lock()
        self.init_solver()

    # 这里可能需要一些额外的保护, 当然外面可能也会进行保护
    def solve(self, req: IKRequest) -> tensor2f:
        if self.solver is None:
            return None

        if req.target_frame is None:
            return None

        # 使用锁保护整个计算过程，防止 set_base 被篡改
        sols = np.zeros((8, self.dof), dtype=np.float64)
        with self.lock:
            # 转换数据...
            joint_seed = np.array(req.joint_seed, dtype=np.float64) if req.joint_seed else np.zeros(self.dof)
            base_pose = ampl.qt7_to_tf44(req.base_frame) if req.base_frame else np.eye(4)
            self.solver.set_base(base_pose)
            end_pose = ampl.qt7_to_tf44(req.target_frame)

            use_iter = False;
            if req.iter_rate is not None and req.iter_number > 0:
                use_iter = True
            
            if use_iter:
                iter_rate = np.array(req.iter_rate, dtype=np.float64)
                status = self.solver.ik_iter(end_pose, joint_seed, sols, iter_rate, req.iter_number)
            else:
                status = self.solver.ik(end_pose, sols) 

        return sols.tolist()

    
    # 根据不同的类型对逆解服务进行初始化
    def init_solver(self):
        if self.robot_type == "fanuc_crx10ia":
            self.dof = 6
            self.solver = ampl.ArmBase("fanuc_crx10ia", ampl.ArmType.CRX6, 6)
        else:
            raise ValueError(f"Unsupport robot type: {self.robot_type}")


# FastApi 服务的实现 

@asynccontextmanager
async def lifespan(app: FastAPI):
    # 初始化全局状态
    app.state.solvers = {}   # 存放所有可能的逆解器
    app.state.lock = asyncio.Lock()
    yield
    app.state.solvers.clear()

app = FastAPI(lifespan=lifespan)

@app.post("/solve_ik", response_model=IKResponse)
async def calculate_ik( request_data: IKRequest, request: Request ):
    solvers: Dict[str, IKSolver] = request.app.state.solvers
    lock: asyncio.Lock = request.app.state.lock
    robot_type = request_data.robot_type

    # 1. 线程安全的逆解器加载 (Double-Check)
    if robot_type not in solvers:
        async with lock:
            if robot_type not in solvers:
                try:
                    solvers[robot_type] = IKSolver(robot_type)
                except Exception as e:
                    return IKResponse(
                        task_id=request_data.task_id,
                        output=[],
                        message=f"初始化机器人 {robot_type} 失败: {str(e)}",
                        code=500
                    )

    solver = solvers[robot_type]

    loop = asyncio.get_event_loop()
    try:
        result = await loop.run_in_executor(None, solver.solve, request_data)
        
        if result is None:
            return IKResponse(task_id=request_data.task_id, output=[], message="Solver error", code=502)

        return IKResponse(
            task_id=request_data.task_id,
            output=result,
            message="success",
            code=0
        )

    except Exception as e:
        return IKResponse(
            task_id=request_data.task_id,
            output=[],
            message=f"计算过程中发生错误: {str(e)}",
            code=501
        )