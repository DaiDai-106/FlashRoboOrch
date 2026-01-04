import viser
import time
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
        self.robots: Dict[str, Dict[str, Any]] = {}  
        

    def add_urdf(self, urdf: URDF):
        name = urdf.name
        base_node_name = f"/user_{self.client.client_id}/base_link_{name}"
        if name in self.robots:
            self.remove_urdf(name)

        frame_handle = self.ui.server.scene.add_frame(base_node_name, show_axes=False)
        viser_urdf_handle = ViserUrdf(self.ui.server, urdf, root_node_name=f"/base_link_{name}")
        self.robots[name] = {
            "frame": frame_handle,
            "visualizer": viser_urdf_handle
        }


    def remove_urdf(self, name: str) -> None:
        if name in self.robots:
            record = self.robots[name]
            record["frame"].remove()
            record["visualizer"].remove()
            del self.robots[name]