import viser
import time
import json
import numpy as np
import ampl
from pathlib import Path
from typing import Dict, Any, Optional, Callable
from robots_orchestra.viz.viser import ViserUI
from viser.extras import ViserUrdf
from yourdfpy import URDF
from robots_orchestra.planner.ik_solver import IKSolver
from robots_orchestra import SCENE_DIR

# 用户会话，负责管理每个用户的所有私有资源
class UserSession:
    def __init__(self, client: viser.ClientHandle, viser_ui: ViserUI):
        self.ui = viser_ui
        self.client = client
        # 用户特定的命名空间前缀
        self.namespace = f"/world/user_{client.client_id}"
        
        # 存储该用户的所有机器人URDF可视化对象
        self.robots: Dict[str, Any] = {}
        
        # 存储该用户每个机器人的base frame（用于设置位置）
        # 结构: {robot_name: frame_handle}
        self.robot_frames: Dict[str, Any] = {}
        
        # 存储该用户每个机器人的关节slider控件
        # 结构: {robot_name: {folder, sliders, actuated_joints}}
        self.robot_sliders: Dict[str, Dict[str, Any]] = {}
        
        # 存储该用户每个机器人的末端执行器轨道工具
        # 结构: {robot_name: {controls, controls_name, ik_solver, current_joint_config}}
        self.end_effector_controls: Dict[str, Dict[str, Any]] = {}
        
        # 存储该用户的IK求解器（按机器人名称）
        self.ik_solvers: Dict[str, IKSolver] = {}
        
        # 场景加载状态标志
        self.is_scene_loaded = False
        
        # 存储该用户的场景加载按钮控件
        self.btn_load_scene: Optional[Any] = None
        self.btn_clear_scene: Optional[Any] = None
        
        # 加载机器人位置配置
        self.robot_config = self.load_robot_config()
        
        # 为每个用户创建独立的场景加载按钮
        self.create_scene_buttons()

    def load_robot_config(self) -> Dict[str, Any]:
        """加载机器人位置配置文件"""
        config_path = SCENE_DIR / "config.json"
        try:
            if config_path.exists():
                with open(config_path, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    return config.get("robot_positions", {})
        except Exception as e:
            print(f"加载机器人配置文件时出错: {e}")
        return {}
    
    def create_scene_buttons(self):
        """为当前用户创建场景加载和清除按钮"""
        # 在场景加载文件夹中为该用户创建按钮（使用唯一的名称）
        button_name_prefix = f"user_{self.client.client_id}_"
        with self.ui.scene_folder:
            self.btn_load_scene = self.ui.server.gui.add_button(
                f"加载场景 (用户 {self.client.client_id})"
            )
            self.btn_clear_scene = self.ui.server.gui.add_button(
                f"清除场景 (用户 {self.client.client_id})"
            )
            # 初始化按钮状态：加载场景可用，清除场景禁用
            self.btn_load_scene.disabled = False
            self.btn_clear_scene.disabled = True
    
    def remove_scene_buttons(self):
        """移除该用户的场景按钮"""
        if self.btn_load_scene is not None:
            try:
                self.btn_load_scene.remove()
            except Exception as e:
                print(f"删除加载场景按钮时出错: {e}")
        if self.btn_clear_scene is not None:
            try:
                self.btn_clear_scene.remove()
            except Exception as e:
                print(f"删除清除场景按钮时出错: {e}")

    def get_robot_position(self, robot_name: str) -> tuple:
        """获取机器人的位置和旋转配置
        
        Returns:
            (position, rotation) 元组，position是(x, y, z)，rotation是四元数(w, x, y, z)
        """
        if robot_name in self.robot_config:
            config = self.robot_config[robot_name]
            position = tuple(config.get("position", [0.0, 0.0, 0.0]))
            rotation = tuple(config.get("rotation", [1.0, 0.0, 0.0, 0.0]))  # 默认无旋转
            return position, rotation
        # 默认位置
        return (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)

    def add_urdf(self, urdf: URDF, on_slider_change: Optional[Callable[[str, np.ndarray], None]] = None):
        """添加URDF机器人并创建相关控件
        
        Args:
            urdf: URDF对象
            on_slider_change: slider值变化时的回调函数，参数为(robot_name, joint_config)
        """
        robot_name = urdf.robot.name  # TODO 这个地方暂时是没有问题的， 如果有多个同样的机械臂可能会有问题
        if robot_name in self.robots:
            self.remove_urdf(robot_name)

        # 获取机器人位置配置
        position, rotation = self.get_robot_position(robot_name)
        
        # 创建机器人的base frame（用于设置位置）
        frame_name = f"{self.namespace}/robot_frame_{robot_name}"
        robot_frame = self.ui.server.scene.add_frame(
            name=frame_name,
            show_axes=False,  # 不显示坐标轴
            position=position,
            wxyz=rotation,  # 四元数 (w, x, y, z)
        )

        self.robot_frames[robot_name] = robot_frame
        # 使用frame作为root节点
        root_node_name = f"{frame_name}/base_link_{robot_name}"
        viser_urdf_handle = ViserUrdf(
            self.ui.server, 
            urdf, 
            root_node_name=root_node_name, 
            load_collision_meshes=False
        ) 
        self.robots[robot_name] = viser_urdf_handle
        
        # 设置默认关节配置
        actuated_joints = urdf.actuated_joints
        dof = len(actuated_joints)
        default_joint = np.array([0.001] * dof, dtype=np.float64)
        self.robots[robot_name].update_cfg(default_joint)
        
        # 创建机器人关节slider控件
        self.create_robot_sliders(robot_name, urdf, on_slider_change)
        
        # 创建末端执行器轨道工具
        self.create_end_effector_orbit_tool(robot_name, urdf, default_joint)

    def remove_urdf(self, robot_name: str) -> None:
        """移除指定的URDF机器人及其所有相关资源"""
        if robot_name not in self.robots:
            return
        
        # 移除末端执行器轨道工具
        if robot_name in self.end_effector_controls:
            self.remove_end_effector_controls(robot_name)
        
        # 移除slider控件
        if robot_name in self.robot_sliders:
            self.remove_robot_sliders(robot_name)
        
        # 移除URDF可视化场景节点
        frame_name = f"{self.namespace}/robot_frame_{robot_name}"
        root_node_name = f"{frame_name}/base_link_{robot_name}"
        try:
            # 删除场景节点（这会删除整个URDF树）
            self.ui.server.scene.remove_by_name(root_node_name)
            # 删除frame（这会删除frame及其所有子节点）
            self.ui.server.scene.remove_by_name(frame_name)
        except Exception as e:
            print(f"删除URDF场景节点时出错: {e}")
        
        del self.robots[robot_name]
        
        # 删除frame引用
        if robot_name in self.robot_frames:
            del self.robot_frames[robot_name]
        
        # 清理IK求解器
        if robot_name in self.ik_solvers:
            del self.ik_solvers[robot_name]

    # TODO 这里还有工具头, 场景中的工件加载, 以及

    def clear_scene(self):
        """清除该用户的所有场景（机器人）"""
        robot_names = list(self.robots.keys())
        for robot_name in robot_names:
            self.remove_urdf(robot_name)
        self.is_scene_loaded = False
        # 更新按钮状态
        if self.btn_load_scene is not None:
            self.btn_load_scene.disabled = False
        if self.btn_clear_scene is not None:
            self.btn_clear_scene.disabled = True
        print(f"用户 {self.client.client_id} 的场景已清除")

    def cleanup(self):
        """清理该用户的所有资源（用户断开时调用）"""
        robot_names = list(self.robots.keys())
        for robot_name in robot_names:
            self.remove_urdf(robot_name)
        self.is_scene_loaded = False
        # 移除该用户的场景按钮
        self.remove_scene_buttons()
        print(f"用户 {self.client.client_id} 的所有资源已清理")

    def create_robot_sliders(
        self, 
        robot_name: str, 
        urdf: URDF,
        on_slider_change: Optional[Callable[[str, np.ndarray], None]] = None
    ):
        """为指定机器人创建所有关节的slider控件"""
        try:
            if robot_name in self.robot_sliders:
                # 如果已存在，先删除旧的
                self.remove_robot_sliders(robot_name)
            
            actuated_joints = urdf.actuated_joints
            
            if len(actuated_joints) == 0:
                print(f"警告: 机器人 {robot_name} 没有可驱动的关节")
                return
            
            # 将Joint对象转换为关节名称字符串列表
            joint_names = []
            for joint in actuated_joints:
                if hasattr(joint, 'name'):
                    joint_names.append(joint.name)
                elif isinstance(joint, str):
                    joint_names.append(joint)
                else:
                    joint_names.append(str(joint))
            
            # 在"机器人拖动"文件夹下创建机器人名称的子文件夹
            with self.ui.robot_drag_folder:
                robot_folder = self.ui.server.gui.add_folder(robot_name)
                joint_sliders = {}
                
                # 定义更新函数，用于所有slider共享
                def update_robot_joints():
                    """更新机器人所有关节配置"""
                    # 获取当前所有关节的值
                    current_config = {}
                    for jn, sl in joint_sliders.items():
                        current_config[jn] = sl.value
                    
                    # 按照joint_names的顺序构建关节配置数组
                    joint_config = np.array([
                        current_config[jn] for jn in joint_names
                    ], dtype=np.float64)
                    
                    # 调用外部回调函数
                    if on_slider_change is not None:
                        on_slider_change(robot_name, joint_config)
                
                # 在robot_folder的上下文中创建所有slider
                with robot_folder:
                    # 为每个关节创建slider
                    for joint_name in joint_names:        
                        lower = -4
                        upper = 4
                        initial_value = 0.0
                        step = 0.01
                        
                        # 创建slider
                        slider = self.ui.server.gui.add_slider(
                            label=joint_name,
                            min=lower,
                            max=upper,
                            step=step,
                            initial_value=initial_value,
                        )
                        joint_sliders[joint_name] = slider
                        
                        # 为每个slider绑定更新回调
                        @slider.on_update
                        def on_slider_update(event: viser.GuiEvent[viser.GuiSliderHandle]):
                            """当slider值变化时，更新机器人关节配置"""
                            update_robot_joints()
                
                self.robot_sliders[robot_name] = {
                    "folder": robot_folder,
                    "sliders": joint_sliders,
                    "actuated_joints": joint_names
                }
                
            print(f"成功为机器人 {robot_name} 创建了 {len(actuated_joints)} 个关节slider")
            
        except Exception as e:
            print(f"创建机器人slider时出错: {e}")
    
    def remove_robot_sliders(self, robot_name: str):
        """移除指定机器人的所有slider控件"""
        try:
            if robot_name not in self.robot_sliders:
                return
            
            robot_info = self.robot_sliders[robot_name]
            
            # 删除folder（删除folder会自动删除其所有子元素，包括slider）
            if "folder" in robot_info:
                try:
                    robot_info["folder"].remove()
                except Exception as e:
                    print(f"删除folder时出错: {e}")
            
            # 从字典中删除
            del self.robot_sliders[robot_name]
            print(f"成功移除机器人 {robot_name} 的slider")
            
        except Exception as e:
            print(f"移除机器人slider时出错: {e}")

    def create_end_effector_orbit_tool(self, robot_name: str, urdf: URDF, joint_config: np.ndarray):
        """在机械臂末端执行器位置创建轨道工具Orbit tool"""
        try:
            links = list(urdf.link_map.keys())
            if not links:
                print(f"警告: 无法找到URDF的链接")
                return
            
            # 找到末端执行器链接
            end_effector_link = urdf.robot.links[-1]
            urdf.update_cfg(joint_config)
            
            # 初始化IK求解器
            if robot_name not in self.ik_solvers:
                try:
                    self.ik_solvers[robot_name] = IKSolver(robot_name)
                    print(f"成功初始化 IK 求解器: {robot_name}")
                except Exception as e:
                    print(f"初始化 IK 求解器失败: {robot_name}, 错误: {e}")
                    return
            
            ik_solver = self.ik_solvers[robot_name]

            # 构建末端执行器链接对应的场景节点名称
            # ViserUrdf 使用 {root_node_name}/visual/{link_path} 格式来创建 mesh 节点
            frame_name = f"{self.namespace}/robot_frame_{robot_name}"
            root_node_name = f"{frame_name}/base_link_{robot_name}"
            
            # 使用与 ViserUrdf 相同的逻辑构建末端执行器链接的节点名称
            # 直接使用末端执行器链接的 mesh 节点作为父节点
            if urdf.scene is not None:
                from viser.extras._urdf import _viser_name_from_frame
                # 使用 ViserUrdf 的内部函数来构建节点名称
                prefixed_root = f"{root_node_name}/visual"
                end_effector_node_name = _viser_name_from_frame(
                    urdf.scene,
                    end_effector_link.name,
                    prefixed_root
                )
            else:
                # 如果没有scene，使用简单的名称
                end_effector_node_name = f"{root_node_name}/visual/{end_effector_link.name}"
            
            # 将轨道工具添加到末端执行器链接的mesh节点下（作为子节点）
            controls_name = f"{end_effector_node_name}/orbit_controls"
            
            # 添加交互式变换控件，作为末端执行器链接的子节点
            # 使用单位变换（轨道工具就在末端执行器位置）
            controls_handle = self.ui.server.scene.add_transform_controls(
                name=controls_name,
                position=(0.0, 0.0, 0.0),  # 相对于父节点的位置（原点）
                wxyz=(1.0, 0.0, 0.0, 0.0),  # 无旋转（单位四元数）
                scale=0.5,
                visible=True,
                disable_axes=False,
                disable_rotations=False,
                disable_sliders=True,
            )
            
            # 存储引用
            self.end_effector_controls[robot_name] = {
                "controls": controls_handle,
                "controls_name": controls_name,
                "robot_name": robot_name,
                "ik_solver": ik_solver,
                "current_joint_config": joint_config.copy(),
            }
            
            # 监听变换控件的更新事件
            @controls_handle.on_update
            def on_controls_update(event: viser.TransformControlsEvent):
                """当用户拖动轨道工具时，进行逆解求解并更新机械臂可视化"""
                try:
                    # 轨道工具现在是末端执行器链接的子节点
                    # new_position 和 new_rotation 是相对于末端执行器链接的
                    new_position_rel = event.target.position
                    new_rotation_rel = event.target.wxyz  # (w, x, y, z)
                    
                    robot_info = self.end_effector_controls[robot_name]
                    ik_solver = robot_info["ik_solver"]
                    current_joint_config = robot_info["current_joint_config"]
                    viser_urdf_handle = self.robots[robot_name]
                    
                    # 获取末端执行器链接当前的变换（相对于base_link）
                    urdf.update_cfg(current_joint_config)
                    tf_end_effector_local = urdf.get_transform(end_effector_link.name)
                    tf_end_effector_local = np.array(tf_end_effector_local, dtype=np.float64, order='C', copy=True)
                    
                    # 构建轨道工具相对于末端执行器链接的变换矩阵
                    # new_rotation_rel 是 (w, x, y, z) 格式，需要转换为 ampl 的 qt7 格式
                    qt7_controls_rel = np.array([
                        new_rotation_rel[1],  # qx
                        new_rotation_rel[2],  # qy
                        new_rotation_rel[3],  # qz
                        new_rotation_rel[0],  # qw
                        new_position_rel[0],  # x
                        new_position_rel[1],  # y
                        new_position_rel[2]   # z
                    ], dtype=np.float64)
                    tf_controls_rel = ampl.qt7_to_tf44(qt7_controls_rel)
                    
                    # 计算轨道工具相对于base_link的绝对变换
                    tf_target_local = tf_end_effector_local @ tf_controls_rel
                    
                    # 提取相对于base_link的局部坐标
                    qt7_target = ampl.tf44_to_qt7(tf_target_local)

                    target_frame = [
                        float(qt7_target[0]),  
                        float(qt7_target[1]),  
                        float(qt7_target[2]),  
                        float(qt7_target[3]),  
                        float(qt7_target[4]),  
                        float(qt7_target[5]),  
                        float(qt7_target[6])   
                    ]
                    
                    # 调用逆解求解
                    solutions = ik_solver.solve(
                        target_frame=target_frame,
                        joint_seed=current_joint_config.tolist(),
                        iter_rate=[1e-3, 1e-2, 0.0, 0.0, 5e-4, 1e-4],
                        iter_number=20,
                    )
                    
                    if solutions is not None and len(solutions) > 0:
                        new_joint_config = np.array(solutions[0], dtype=np.float64)  # TODO 这个地方对于逆解的选取可能需要有些说法 
                        viser_urdf_handle.update_cfg(new_joint_config)
                        robot_info["current_joint_config"] = new_joint_config
                        print(f"用户 {self.client.client_id} 的机器人 {robot_name} 逆解成功")
                    else:
                        print(f"用户 {self.client.client_id} 的机器人 {robot_name} 逆解失败，无解")
                        
                except Exception as e:
                    print(f"更新机械臂可视化时出错: {e}")
            
        except Exception as e:
            print(f"创建末端执行器轨道工具时出错: {e}")
    
    def remove_end_effector_controls(self, robot_name: str):
        """移除指定机器人的末端执行器轨道工具"""
        if robot_name not in self.end_effector_controls:
            return
        
        # 移除场景中的控件（通过删除场景节点）
        robot_info = self.end_effector_controls[robot_name]
        controls_name = robot_info["controls_name"]
        try:
            # 尝试通过场景API删除
            self.ui.server.scene.remove_by_name(controls_name)
        except Exception as e:
            print(f"删除末端执行器控件时出错: {e}")
        
        del self.end_effector_controls[robot_name]