import numpy as np


# 目前的计划是 轴运动使用距离场的规划, 笛卡尔的路径则是离线编程即可.
class Planner:
    def __init__(self, robot_name: str):
        self.robot_name = robot_name
        self.planner = None

    def plan(self, target_frame: np.ndarray):
        pass

    # 线性插值的轨迹
    @staticmethod
    def sample_trajectory(
        q_start: np.array,
        q_end: np.array,
        n: int = 10,
        include_start=False,
        inclue_end=True,
    ) -> np.array:
        return np.linspace(q_start, q_end, n + 2, endpoint=True)[
            not include_start : n + 2 - (not inclue_end)
        ]


# TODO 是否会在这里接入距离场的规划