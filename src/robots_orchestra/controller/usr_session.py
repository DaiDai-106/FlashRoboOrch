import viser
import time
import numpy as np
from typing import Dict
from robots_orchestra.viz.viser import ViserUI
from viser.extras import ViserUrdf
from typing import Dict, Optional, Any
from yourdfpy import URDF

# 用户绘画, 负责存每个用户私有的东西
class UserSession:
    def __init__(self, client: viser.ClientHandle, viser_ui: ViserUI):
        self.ui = viser_ui
        self.client = client
        self.robots: Dict[str, Any] = {}  

    def add_urdf(self, urdf: URDF):
        name = urdf.robot.name
        if name in self.robots:
            self.remove_urdf(name)

        viser_urdf_handle = ViserUrdf(self.ui.server, urdf, root_node_name=f"/world/origin/base_link_{name}", load_collision_meshes = False) 
        self.robots[name] = viser_urdf_handle
        actuated_joints = urdf.actuated_joints
        dof = len(actuated_joints)  # 获取当前机械臂的自由度 
        default_joint = np.array([0.001] * dof, dtype=np.float64)
        self.robots[name].update_cfg(default_joint)

    def remove_urdf(self, name: str) -> None:
        if name in self.robots:
            del self.robots[name]