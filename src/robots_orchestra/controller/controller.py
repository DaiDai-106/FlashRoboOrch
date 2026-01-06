import viser
import os
import numpy as np
from yourdfpy import URDF
from typing import Dict, Any
import ampl

from robots_orchestra import SCENE_DIR
from robots_orchestra.controller.usr_session import UserSession
from robots_orchestra.viz.viser import ViserUI

class Controller:
    def __init__(self, viser_ui: ViserUI):
        self.ui = viser_ui
        self.sessions: Dict[viser.ClientHandle, UserSession] = {}

        # 存储每个用户每个机器人的末端执行器轨道工具（Orbit tool）
        self.end_effector_controls: Dict[viser.ClientHandle, Dict[str, Dict[str, Any]]] = {}

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
            # 清理该用户的末端执行器轨道工具
            self.remove_all_end_effector_controls(client)
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
            
            # 在末端执行器位置创建轨道工具（Orbit tool）
            robot_name = urdf.robot.name
            actuated_joints = urdf.actuated_joints
            dof = len(actuated_joints)
            default_joint = np.array([0.001] * dof, dtype=np.float64)
            self.create_end_effector_orbit_tool(client, robot_name, urdf, default_joint)
            
            print(f"用户 {client.client_id} 上传了 URDF: {file_name}")
        except Exception as e:
            print(f"加载 URDF 时出错: {e}")
            import traceback
            traceback.print_exc()


    def create_end_effector_orbit_tool(self, client: viser.ClientHandle, robot_name: str, urdf: URDF, joint_config: np.ndarray):
        """在机械臂末端执行器位置创建轨道工具Orbit tool, 支持平移和旋转控制"""
        try:
            links = list(urdf.link_map.keys())
            print(f"links: {links}")
            if not links:
                print(f"警告: 无法找到URDF的链接")
                return
            
            # 尝试找到末端执行器链接（通常是最后一个非base_link的链接）
            end_effector_link = urdf.robot.links[-1]
            # 使用yourdfpy计算正运动学，获取末端执行器位置
            urdf.update_cfg(joint_config)
            
            # 获取末端执行器的变换矩阵
            tf = urdf.get_transform(end_effector_link.name)
            tf = np.array(tf, dtype=np.float64, order='C', copy=True)
            print(f"tf: {tf}")
        
            position = tf[:3, 3]  # 提取位置    
            # 使用 ampl 将 4x4 变换矩阵转换为 7 维位姿 (x, y, z, qx, qy, qz, qw)
            qt7 = ampl.tf44_to_qt7(tf)
            quaternion = (qt7[0], qt7[1], qt7[2], qt7[3])  

            # 创建轨道工具名称（包含用户ID和机器人名称）
            controls_name = f"/world/origin/end_effector_orbit_{client.client_id}_{robot_name}"
            
            # 添加交互式变换控件（Orbit tool，支持平移和旋转）
            controls_handle = self.ui.server.scene.add_transform_controls(
                name=controls_name,
                position=position,
                wxyz=quaternion,  # 四元数 (w, x, y, z)
                scale=0.5,
                visible=True,
                disable_axes=False,  # 启用轴控制（平移）
                disable_rotations=False,  # 启用旋转控制
                disable_sliders=True,  # 启用平面滑块控制
            )
            
            # 存储引用
            if client not in self.end_effector_controls:
                self.end_effector_controls[client] = {}
            
            self.end_effector_controls[client][robot_name] = {
                "controls": controls_handle,
                "controls_name": controls_name
            }
            
            # 监听变换控件的更新事件
            @controls_handle.on_update
            def on_controls_update(event: viser.TransformControlsEvent):
                """当用户拖动轨道工具时，更新末端执行器位置和旋转"""
                new_position = event.target.position
                new_rotation = event.target.wxyz  # 四元数 (w, x, y, z)
                
            
        except Exception as e:
            print(f"创建末端执行器轨道工具时出错: {e}")
    
    def remove_all_end_effector_controls(self, client: viser.ClientHandle):
        """移除指定用户的所有末端执行器轨道工具"""
        if client not in self.end_effector_controls:
            return
        
        for robot_name, objects in self.end_effector_controls[client].items():
            controls_name = objects["controls_name"]
            
            # 移除轨道工具控件
            if controls_name in self.ui.server.scene.nodes:
                del self.ui.server.scene.nodes[controls_name]
        
        del self.end_effector_controls[client]

    # 运行控制器
    def run(self):
        self.ui.run()