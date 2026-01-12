import viser
import os
import json
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

        # 不再使用全局按钮，改为在UserSession中为每个用户创建独立按钮

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
        client = session.client
        
        if session.is_scene_loaded:
            return  # 如果场景已经加载，不允许再次加载
        
        try:
            config_path = SCENE_DIR / "config.json"
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
            
            robot_positions = config.get("robot_positions", {})
            
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
            
            # 加载配置中的所有机器人
            for robot_name in robot_positions.keys():
                urdf_path = SCENE_DIR / "urdf" / f"{robot_name}.urdf"
                if urdf_path.exists():
                    urdf = URDF.load(str(urdf_path))
                    session.add_urdf(urdf, on_slider_change=on_slider_change)
                    print(f"用户 {client.client_id} 加载了机器人: {robot_name}")



            # TODO 这里还用工具头和工件的加载
            

            
            # 标记场景已加载
            session.is_scene_loaded = True
            
            # 禁用加载场景按钮，启用清除场景按钮
            if session.btn_load_scene is not None:
                session.btn_load_scene.disabled = True
            if session.btn_clear_scene is not None:
                session.btn_clear_scene.disabled = False
            
            print(f"用户 {client.client_id} 场景加载成功")
        except Exception as e:
            print(f"加载场景时出错: {e}")

    def handle_clear_scene(self, session: UserSession, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理清除场景按钮事件"""
        client = session.client
        
        if not session.is_scene_loaded:
            return  # 如果场景未加载，不允许清除
    
        session.clear_scene() #清空加载的场景

    def run(self):
        """运行控制器"""
        self.ui.run()