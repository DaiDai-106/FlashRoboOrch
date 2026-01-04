import viser
import time
from typing import List, Dict, Any
from pathlib import Path

# 这里只负责 UI 的设计
class ViserUI:
    def __init__(self, title: str = "Robots Orchestra", port: int = 8080):
        self.server = viser.ViserServer( port=port, label=title)  # 局域网监听端口
        self.urdf_dropdowns: List[Dict[str, Any]] = []
        self.initialize_ui()  # 初始化 UI 设计

    def initialize_ui(self):
        # 配置美观的背景灯光效果
    
        # 添加默认网格 (grid) - XY 平面，适合机器人场景
        self.server.scene.add_grid(
            name="/world",
            width=10.0,
            height=10.0,
            plane="xy",  # XY 平面，Z 轴向上
            cell_size=0.5,
            section_size=1.0,
            section_color=[0, 0, 0],
            cell_color=[128, 128, 128],
            cell_thickness=0.5,
            infinite_grid=False,  # 无限网格
        )
        
        # 在原点 (0, 0, 0) 显示坐标系
        self.server.scene.add_frame(
            name="/world/origin",
            show_axes=True,
            axes_length=1.0,  # 坐标轴长度
            axes_radius=0.02,  # 坐标轴半径
            position=(0.0, 0.0, 0.0)  # 原点位置
        )

        self.server.gui.configure_theme(control_width="large")
        
        # 创建"场景加载"文件夹并保存引用
        self.scene_folder = self.server.gui.add_folder("场景加载")
        with self.scene_folder:
            self.btn_upload = self.server.gui.add_upload_button(
                "上传 URDF",
                mime_type="application/xml,text/xml,.urdf"  # 限制上传类型为 URDF
            )
            

    @staticmethod
    def run():
        while True:
            time.sleep(0.1)