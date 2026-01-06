from typing import Dict, Optional, Set, Any
import asyncio
import json
import numpy as np
from pydantic import ValidationError
from contextlib import asynccontextmanager
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from robots_orchestra.services.class_def import *
from robots_orchestra.planner.ik_solver import IKSolver

# FastApi 服务的实现 
@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.solvers = {}   # 存放所有可能的逆解器
    app.state.lock = asyncio.Lock()
    yield
    app.state.solvers.clear()

app = FastAPI(
    title="Robots Orchestra 服务",
    description="提供机器人逆解计算的 WebSocket 服务，支持多客户端并发连接",
    version="0.1.0",
    lifespan=lifespan
)


@app.get("/status")
async def status():
    """查询服务状态"""
    return {
        "status": "running",
        "solvers_loaded": list(app.state.solvers.keys())
    }


"""发送 WebSocket 响应的辅助函数"""
async def send_response(
    websocket: WebSocket,
    task_id: str,
    code: StatusCode,
    data: Optional[Dict[str, Any]] = None,
    message: Optional[str] = None
) -> None:
    try:
        response = WsResponseEnvelope(
            task_id=task_id,
            code=code.value if isinstance(code, StatusCode) else code,
            data=data,
            message=message
        )
        await websocket.send_json(response.model_dump())
    except Exception as e:
        print(f"发送响应失败: {e}")

""" websocket 总服务, 目前是基于任务分发式的实现 """
@app.websocket("/ws/{client_id}")
async def websocket_endpoint(websocket: WebSocket, client_id: str):
    await websocket.accept()

    envelope = None  # 初始化 envelope，避免在异常处理中未定义
    while True:
        try:
            raw_text = await websocket.receive_text()
            # 拆信封
            try:
                envelope = WsRequestEnvelope.model_validate_json(raw_text)
            except Exception as e:
                task_id = "unknown"
                try:
                    json_data = json.loads(raw_text)
                    task_id = json_data.get("task_id", "unknown")
                except:
                    pass
                await send_response(
                    websocket=websocket,
                    task_id=task_id,
                    code=StatusCode.BAD_REQUEST,
                    data = None,
                    message=f"请求格式错误: {str(e)}"
                )
                continue

            # 2. 任务分发
            if envelope.command == "solve_ik":
                try:
                    # 验证 payload 并转换为 WsIkRequest
                    ik_request = WsIkRequest(**envelope.payload)
                    robot_type = ik_request.robot_type
    
                    solver = None
                    
                    if robot_type in websocket.app.state.solvers:
                        solver = websocket.app.state.solvers[robot_type]
                    else:
                        async with websocket.app.state.lock:
                            # Double-Check 模式
                            if robot_type not in websocket.app.state.solvers:
                                try:
                                    new_solver = IKSolver(robot_type)
                                    websocket.app.state.solvers[robot_type] = new_solver
                                except Exception as e:
                                    await send_response(
                                        websocket=websocket,
                                        task_id=envelope.task_id,
                                        code=StatusCode.SERVER_ERROR,
                                        data = None,
                                        message=f"初始化机器人 {robot_type} 失败: {str(e)}"
                                    )
                                    continue
                            solver = websocket.app.state.solvers[robot_type]

                    # 注意：因为 solver 对象在内存里，我们用 asyncio.to_thread 放入线程池
                    result_output = await asyncio.to_thread(
                        solver.solve, 
                        target_frame=ik_request.target_frame, 
                        joint_seed=ik_request.joint_seed,
                        iter_rate=ik_request.iter_rate,
                        iter_number=ik_request.iter_number
                    )

                    if result_output is not None:
                        await send_response(
                            websocket=websocket,
                            task_id=envelope.task_id,
                            data=WsIkResponse(output=result_output).model_dump(),
                            code=StatusCode.SUCCESS,
                            message="逆解计算成功"
                        )
                    else:
                        # 无解情况
                        await send_response(
                            websocket=websocket,
                            task_id=envelope.task_id,
                            code=StatusCode.IK_NO_SOLUTION,
                            data = None,
                            message="未找到逆解：目标点可能不可达"
                        )
                        
                except ValidationError as e:
                    # 参数验证错误
                    await send_response(
                        websocket=websocket,
                        task_id=envelope.task_id,
                        data = None,
                        code=StatusCode.VALIDATION_ERROR,
                        message=f"参数验证失败: {str(e)}"
                    )
                except Exception as e:
                    # 其他计算错误
                    await send_response(
                        websocket=websocket,
                        task_id=envelope.task_id,
                        code=StatusCode.SERVER_ERROR,
                        data = None,
                        message=f"计算过程中发生错误: {str(e)}"
                    )

            elif envelope.command == "heartbeat":
                await send_response(
                    websocket=websocket,
                    task_id=envelope.task_id,
                    data={"type": "pong"},
                    code=StatusCode.SUCCESS,
                    message="pong"
                )
            else:
                await send_response(
                    websocket=websocket,
                    task_id=envelope.task_id,
                    code=StatusCode.UNKNOWN_COMMAND,
                    data = None,
                    message=f"未知的命令: {envelope.command}"
                )

        except WebSocketDisconnect:
            print(f"客户端 {client_id} 断开连接")
            break
        except Exception as e:
            task_id = envelope.task_id if envelope else "unknown"
            await send_response(
                websocket=websocket,
                task_id=task_id,
                code=StatusCode.SERVER_ERROR,
                message=f"服务器内部错误: {str(e)}"
            )
