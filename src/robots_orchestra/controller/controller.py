import viser
import os
import numpy as np
from yourdfpy import URDF
from typing import Dict

from robots_orchestra import SCENE_DIR
from robots_orchestra.controller.usr_session import UserSession
from robots_orchestra.viz.viser import ViserUI

class Controller:
    """控制器：负责事件分发和协调各组件"""
    def __init__(self, viser_ui: ViserUI):
        self.ui = viser_ui
        self.sessions: Dict[viser.ClientHandle, UserSession] = {}

        @self.ui.server.on_client_connect
        def _(client: viser.ClientHandle):
            self.handle_connect(client)
            self.set_default_camera(client)

        @self.ui.server.on_client_disconnect
        def _(client: viser.ClientHandle):
            self.handle_disconnect(client)

        @self.ui.btn_upload.on_upload
        def on_urdf_upload(event: viser.GuiEvent[viser.GuiUploadButtonHandle]):
            self.handle_upload_urdf(event)

    #----- UI 事件处理 ------------------------------------------------------------

    def handle_connect(self, client: viser.ClientHandle):
        """处理客户端连接事件"""
        print(f"用户 {client.client_id} 上线")
        self.sessions[client] = UserSession(client, self.ui)

    def handle_disconnect(self, client: viser.ClientHandle):
        """处理客户端断开事件"""
        if client in self.sessions:
            # 清理该用户的所有资源（由UserSession统一管理）
            self.sessions[client].cleanup()
            del self.sessions[client]
            print(f"用户 {client.client_id} 下线")


    def set_default_camera(self, client: viser.ClientHandle):
        """设置默认相机视角"""
        client.camera.position = (3.0, 3.0, 3.0)
        client.camera.look_at = (0.0, 0.0, 0.0)
        client.camera.up_direction = (0.0, 0.0, 1.0)

    def handle_upload_urdf(self, event: viser.GuiEvent[viser.GuiUploadButtonHandle]):
        """处理URDF文件上传事件"""
        client = event.client
        if client is None or client not in self.sessions:
            return
        
        # 从事件中获取上传的文件
        uploaded_file: viser.UploadedFile = event.target.value
        if uploaded_file is None:
            return

        # UploadedFile 只有 name 和 content，需要将内容写入临时文件来获取文件路径
        file_name = uploaded_file.name
        file_path = os.path.join(SCENE_DIR, "urdf", file_name)
        
        try:
            urdf = URDF.load(file_path)
            print(f"成功加载 URDF, 文件路径: {file_path}")
            
            # 获取用户的 session
            session = self.sessions[client]
            
            # 定义slider变化回调函数
            def on_slider_change(robot_name: str, joint_config: np.ndarray):
                """当slider值变化时, 更新URDF的关节配置"""
                try:
                    if robot_name in session.robots:
                        viser_urdf_handle = session.robots[robot_name]
                        viser_urdf_handle.update_cfg(joint_config)
                        print(f"用户 {client.client_id} 的机器人 {robot_name} 关节配置已更新")
                except Exception as e:
                    print(f"更新机器人关节配置时出错: {e}")
            
            # 委托给UserSession处理（它会创建所有相关资源）
            session.add_urdf(urdf, on_slider_change=on_slider_change)
            
            print(f"用户 {client.client_id} 上传了 URDF: {file_name}")
        except Exception as e:
            print(f"加载 URDF 时出错: {e}")

    def run(self):
        """运行控制器"""
        self.ui.run()