import ampl
import numpy as np
import threading
from typing import Optional
from data_model import tensor1f, tensor2f

class IKSolver:
    def __init__(self, robot_type: str):
        self.robot_type = robot_type
        self.solver = None
        self.dof = None
        self.lock = threading.Lock()
        self.init_solver()

    # 逆解求解的主接口
    def solve(self, target_frame: tensor1f, joint_seed: Optional[tensor1f] = None, iter_rate: Optional[tensor1f] = None, iter_number: Optional[int] = 20) -> Optional[tensor2f]:
        if self.solver is None:
            return None

        if len(target_frame) != 7:
            return None

        target_endpose = np.array(target_frame, dtype=np.float64)
        sols = np.zeros((8, self.dof), dtype=np.float64)
        with self.lock:
            joint_seed = np.array(joint_seed, dtype=np.float64) if joint_seed else np.zeros(self.dof)
            self.solver.set_base(np.eye(4))
            end_pose = ampl.qt7_to_tf44(target_endpose)

            use_iter = False
            if iter_rate is not None and iter_number is not None and iter_number > 0:
                use_iter = True

            if self.robot_type != "fanuc_crx10ia":
                use_iter = False
            
            if use_iter:
                iter_rate = np.array(iter_rate, dtype=np.float64)
                iter_number = iter_number if iter_number else 20
                status = self.solver.ik_iter(end_pose, joint_seed, sols, iter_rate, iter_number)
            else:
                status = self.solver.ik(end_pose, sols) 


            sols = sols.tolist()
            if self.robot_type == "abb_irb6700_150_320":
                if len(sols) >= 2:
                    sols[0], sols[2] = sols[2], sols[0]

        return sols

    
    # 根据不同的类型对逆解服务进行初始化
    def init_solver(self):
        if self.robot_type == "fanuc_crx10ia":
            self.dof = 6
            self.solver = ampl.ArmBase("fanuc_crx10ia", ampl.ArmType.CRX6, 6)
        elif self.robot_type == "abb_irb6700_150_320":
            self.dof = 6
            self.solver = ampl.ArmBase("abb_irb6700_150_320", ampl.ArmType.Industrial6, 6)
        else:
            raise ValueError(f"Unsupport robot type: {self.robot_type}")