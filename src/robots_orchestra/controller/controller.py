import viser
from typing import Dict
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

        # 全局按钮事件处理（从event中获取client信息）
        @self.ui.abb_offline_planning.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_abb_offline_planning(event)
        
        
        # 焊缝定位拍照
        @self.ui.franka_capture.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_franka_capture(event)

        # Marvin左臂拍照任务
        @self.ui.left_marvin_capture.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_capture(event)

    #----- UI 事件处理 ------------------------------------------------------------

    def handle_connect(self, client: viser.ClientHandle):
        """处理客户端连接事件"""
        print(f"用户 {client.client_id} 上线")
        session = UserSession(client, self.ui)
        self.sessions[client] = session
        self.setup_user_button_handlers(session) # 为当前用户设置按钮事件处理

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

    def setup_user_button_handlers(self, session: UserSession):
        """为用户会话设置按钮事件处理"""
        client = session.client
        @session.btn_load_scene.on_click
        def on_load_scene_click(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_load_scene(session, event)
        @session.btn_clear_scene.on_click
        def on_clear_scene_click(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_clear_scene(session, event)

    def handle_load_scene(self, session: UserSession, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理加载场景按钮事件"""
        # 直接调用session的load_scene方法，所有逻辑都在UserSession内部处理
        session.load_scene()

    def handle_clear_scene(self, session: UserSession, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理清除场景按钮事件"""
        client = session.client
        
        if not session.is_scene_loaded:
            return  # 如果场景未加载，不允许清除
    
        session.clear_scene() #清空加载的场景

    def handle_abb_offline_planning(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理ABB框架移动按钮事件（全局按钮，从event中获取触发事件的客户端）"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.abb_offline_planning()



    def handle_franka_capture(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理焊接定位拍照按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.franka_capture()

    def handle_left_marvin_capture(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂拍照按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_capture()

    def run(self):
        """运行控制器"""
        self.ui.run()