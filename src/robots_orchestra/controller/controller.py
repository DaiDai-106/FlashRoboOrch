import viser
import os
import numpy as np
from yourdfpy import URDF
from typing import Dict

from robots_orchestra import SCENE_DIR
from robots_orchestra.controller.usr_session import UserSession
from robots_orchestra.viz.viser import ViserUI

class Controller:
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

    # 新用户进来，给他开个户 (Session)
    def handle_connect(self, client: viser.ClientHandle):
        print(f"用户 {client.client_id} 上线")
        self.sessions[client] = UserSession(client, self.ui)

    # 用户下线，销户
    def handle_disconnect(self, client: viser.ClientHandle):
        if client in self.sessions:
            del self.sessions[client]
            print(f"用户 {client.client_id} 下线")


    # 设置默认相机视角 - 当客户端连接时
    def set_default_camera(self, client: viser.ClientHandle):
        client.camera.position = (3.0, 3.0, 3.0)
        client.camera.look_at = (0.0, 0.0, 0.0)
        client.camera.up_direction = (0.0, 0.0, 1.0)

    # 处理 URDF 文件上传
    def handle_upload_urdf(self, event: viser.GuiEvent[viser.GuiUploadButtonHandle]):
        client = event.client
        if client is None or client not in self.sessions:
            return
        
        # 从事件中获取上传的文件
        uploaded_file: viser.UploadedFile = event.target.value
        if uploaded_file is None:
            return

        # UploadedFile 只有 name 和 content，没有绝对路径
        # 需要将内容写入临时文件来获取文件路径
        file_name = uploaded_file.name
        file_path = os.path.join(SCENE_DIR, "urdf", file_name)
        
        try:
            urdf = URDF.load(file_path)
            print(f"成功加载 URDF, 文件路径: {file_path}")
            # 获取用户的 session 并添加 URDF
            session = self.sessions[client]
            session.add_urdf(urdf)
            print(f"用户 {client.client_id} 上传了 URDF: {file_name}")
        except Exception as e:
            print(f"加载 URDF 时出错: {e}")


    # 运行控制器
    def run(self):
        self.ui.run()