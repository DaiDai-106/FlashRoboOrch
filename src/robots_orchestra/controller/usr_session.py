import viser
import time
import json
import numpy as np
import ampl
import trimesh
from pathlib import Path
from typing import Dict, Any, Optional, Callable
from robots_orchestra.viz.viser import ViserUI
from viser.extras import ViserUrdf
from yourdfpy import URDF
from robots_orchestra.planner.ik_solver import IKSolver
from robots_orchestra.planner.planner import Planner
from robots_orchestra import SCENE_DIR
from viser.extras._urdf import _viser_name_from_frame

# 用户会话，负责管理每个用户的所有私有资源
class UserSession:
    def __init__(self, client: viser.ClientHandle, viser_ui: ViserUI):
        self.ui = viser_ui
        self.client = client


        self.is_scene_loaded = False # 场景加载状态标志

        self.namespace = f"/world/user_{client.client_id}" # 用户特定的命名空间前缀
        self.robots: Dict[str, Any] = {} # 存储该用户的所有机器人URDF可视化对象
        self.mobile_cars: Dict[str, Any] = {} # 存储该用户的所有移动小车可视化对象
        self.objects: Dict[str, Any] = {} # 存储该用户的所有对象可视化对象

        self.robot_frames: Dict[str, Any] = {} # 存储该用户每个机器人的基座base frame（用于设置位置）
        self.mobile_car_frames: Dict[str, Any] = {} # 存储该用户每个移动小车的基座base frame（用于设置位置）
        self.robot_sliders: Dict[str, Dict[str, Any]] = {} # 存储该用户每个机器人的关节slider控件, 结构: {robot_name: {folder, sliders, actuated_joints}}
        self.end_effector_controls: Dict[str, Dict[str, Any]] = {} # 存储该用户每个机器人的末端执行器轨道工具, 结构: {robot_name: {controls, controls_name, ik_solver, current_joint_config}}
        self.ik_solvers: Dict[str, IKSolver] = {} # 存储该用户的IK求解器（按机器人名称）
        
        # 存储规划轨迹（用于仿真回放）
        self.abb_trajectory: Optional[np.ndarray] = None  # 形状: (N, dof) - N个时间步，每个时间步的关节配置
        self.abb_robot_name: Optional[str] = None  # 执行轨迹的机器人名称
        self.abb_simulation_slider: Optional[Any] = None  # 仿真进度条控件
        self.planners: Dict[str, Planner] = {} # 存储该用户的规划器（按机器人名称）

        self.robot_config = self.load_robot_config() # 加载机器人位置配置
        self.mobile_car_config = self.load_mobile_car_config() # 加载移动小车位置配置
        self.objects_config = self.load_objects_config() # 加载对象位置配置
    
        # 初始化部分功能
        self.btn_load_scene: Optional[Any] = None # 存储该用户的场景加载按钮控件
        self.btn_clear_scene: Optional[Any] = None # 存储该用户的场景清除按钮控件
        self.create_scene_buttons() # 为每个用户创建独立的场景加载按钮
    
    # --------------------------------------------------------- 按钮创建和移除 ---------------------------------------------------------
    def create_scene_buttons(self):
        """为当前用户创建场景加载和清除按钮"""
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

    # --------------------------------------------------------- 基础操作 ---------------------------------------------------------
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

    def load_mobile_car_config(self) -> Dict[str, Any]:
        """加载移动小车位置配置文件"""
        config_path = SCENE_DIR / "config.json"
        try:
            if config_path.exists():
                with open(config_path, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    return config.get("mobile_car", {})
        except Exception as e:
            print(f"加载移动小车配置文件时出错: {e}")
        return {}

    def load_objects_config(self) -> Dict[str, Any]:
        """加载对象位置配置文件"""
        config_path = SCENE_DIR / "config.json"
        try:
            if config_path.exists():
                with open(config_path, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    return config.get("objects", {})
        except Exception as e:
            print(f"加载对象配置文件时出错: {e}")

    def get_position(self, entity_name: str, entity_type: str) -> tuple:
        """获取机器人、小车或对象的位置和旋转配置
        
        Args:
            entity_name: 实体名称（可以是机器人名称、小车名称或对象名称）
            entity_type: 实体类型 ("robot", "mobile_car", "object")
        
        Returns:
            (position, rotation) 元组，position是(x, y, z)，rotation是四元数(w, x, y, z)
        """
        if entity_type == "robot":
            if entity_name not in self.robot_config:
                return (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
            config = self.robot_config[entity_name]
            position = tuple(config.get("position", [0.0, 0.0, 0.0]))
            rotation = tuple(config.get("rotation", [1.0, 0.0, 0.0, 0.0]))  # 默认无旋转
            return position, rotation
        elif entity_type == "mobile_car":
            if entity_name not in self.mobile_car_config:
                return (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
            config = self.mobile_car_config[entity_name]
            position = tuple(config.get("position", [0.0, 0.0, 0.0]))
            rotation = tuple(config.get("rotation", [1.0, 0.0, 0.0, 0.0]))  # 默认无旋转
            return position, rotation
        elif entity_type == "object":
            if entity_name not in self.objects_config:
                return (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
            config = self.objects_config[entity_name]
            position = tuple(config.get("position", [0.0, 0.0, 0.0]))
            rotation = tuple(config.get("rotation", [1.0, 0.0, 0.0, 0.0]))  # 默认无旋转
            return position, rotation
    
        return (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)

    # --------------------------------------------------------- 场景中全部物体初的一次性的加载 ---------------------------------------------------------
    def add_urdf(self, urdf: URDF, entity_type: str, on_slider_change: Optional[Callable[[str, np.ndarray], None]] = None):
        """添加URDF机器人或移动小车并创建相关控件
        
        Args:
            urdf: URDF对象
            entity_type: 实体类型 ("robot", "mobile_car")
            on_slider_change: slider值变化时的回调函数,参数为(robot_name, joint_config)
        """
        entity_name = urdf.robot.name  # 实体名称（机器人或小车名称）
        
        # 根据entity_type决定存储位置和是否已存在
        if entity_type == "robot":
            storage_dict = self.robots
            if entity_name in self.robots or entity_name in self.mobile_cars:
                existing_type = "robot" if entity_name in self.robots else "mobile_car"
                self.remove_urdf(entity_name, existing_type)
        elif entity_type == "mobile_car":
            storage_dict = self.mobile_cars
            if entity_name in self.mobile_cars or entity_name in self.robots:
                existing_type = "mobile_car" if entity_name in self.mobile_cars else "robot"
                self.remove_urdf(entity_name, existing_type)
        else:
            print(f"警告: 未知的实体类型 {entity_type}")
            return

        # 获取位置配置（根据entity_type）
        position, rotation = self.get_position(entity_name, entity_type)

        frame_name = None
        # 创建实体的base frame（用于设置位置）
        if entity_type == "robot" and "attached_car" in self.robot_config[entity_name].keys():
            attached_car_name = self.robot_config[entity_name]["attached_car"]
            if attached_car_name in self.mobile_cars:
                attached_car_urdf = self.mobile_cars[attached_car_name]._urdf
                end_effector_link = attached_car_urdf.robot.links[-1]
                if attached_car_urdf.scene is not None:
                    prefixed_root = f"{self.namespace}/mobile_car_frame_{attached_car_name}/{attached_car_name}/visual"
                    end_effector_node_name = _viser_name_from_frame(
                        attached_car_urdf.scene,
                        end_effector_link.name,
                        prefixed_root
                    )
                    frame_name = f"{end_effector_node_name}/{entity_type}_frame_{entity_name}"
            else:
                print(f"警告: 移动小车 {attached_car_name} 不存在")
                return
        else:
            frame_name = f"{self.namespace}/{entity_type}_frame_{entity_name}"  # 使用统一的命名格式

        entity_frame = self.ui.server.scene.add_frame(
            name=frame_name,
            show_axes=False,  # 不显示坐标轴
            position=position,
            wxyz=rotation,  # 四元数 (w, x, y, z)
        )

        if entity_type == "robot":
            self.robot_frames[entity_name] = entity_frame
        elif entity_type == "mobile_car":
            self.mobile_car_frames[entity_name] = entity_frame
       
        # 使用frame作为root节点
        root_node_name = f"{frame_name}/{entity_name}"
        viser_urdf_handle = ViserUrdf(
            self.ui.server, 
            urdf, 
            root_node_name=root_node_name, 
            load_collision_meshes=False
        ) 
        
        # 根据entity_type存储到对应的字典
        storage_dict[entity_name] = viser_urdf_handle
        actuated_joints = urdf.actuated_joints
        dof = len(actuated_joints)
        default_joint = np.array([0.001] * dof, dtype=np.float64)
        viser_urdf_handle.update_cfg(default_joint)
        
        # 根据entity_type决定创建哪些控件
        if entity_type == "robot":
            # 机器人：创建所有控件（slider和末端执行器工具）
            self.create_end_effector_orbit_tool(entity_name, urdf, default_joint)
            self.create_robot_sliders(entity_name, urdf, on_slider_change)
        elif entity_type == "mobile_car":
            pass

    def remove_urdf(self, entity_name: str, entity_type: str) -> None:
        """移除指定的URDF机器人或移动小车及其所有相关资源
        
        Args:
            entity_name: 实体名称（机器人或小车名称）
            entity_type: 实体类型 ("robot", "mobile_car")
        """
        # 检查是否在robots或mobile_cars中
        in_robots = entity_name in self.robots
        in_mobile_cars = entity_name in self.mobile_cars
        
        if not in_robots and not in_mobile_cars:
            return
        
        # 移除末端执行器轨道工具（仅机器人有）
        if entity_name in self.end_effector_controls:
            self.remove_end_effector_controls(entity_name)
        
        # 移除slider控件
        if entity_name in self.robot_sliders:
            self.remove_robot_sliders(entity_name)
        
        # 移除URDF可视化场景节点
        frame_name = f"{self.namespace}/{entity_type}_frame_{entity_name}"
        root_node_name = f"{frame_name}/{entity_name}"
        try:
            # 删除场景节点（这会删除整个URDF树）
            self.ui.server.scene.remove_by_name(root_node_name)
            # 删除frame（这会删除frame及其所有子节点）
            self.ui.server.scene.remove_by_name(frame_name)
        except Exception as e:
            print(f"删除URDF场景节点时出错: {e}")
        
        # 从对应的存储字典中删除
        if in_robots:
            del self.robots[entity_name]
        if in_mobile_cars:
            del self.mobile_cars[entity_name]
        
        # 删除frame引用（根据entity_type删除对应的frame字典）
        if entity_type == "robot" and entity_name in self.robot_frames:
            del self.robot_frames[entity_name]
        elif entity_type == "mobile_car" and entity_name in self.mobile_car_frames:
            del self.mobile_car_frames[entity_name]
        
        # 清理IK求解器（仅机器人有）
        if entity_type == "robot" and entity_name in self.ik_solvers:
            del self.ik_solvers[entity_name]

        # 清理规划器（仅机器人有）
        if entity_type == "robot" and entity_name in self.planners:
            del self.planners[entity_name]

    def load_mobile_car(self, car_name: str):
        """加载移动小车（使用URDF方式）"""
        try:
            # 构建移动小车URDF路径
            urdf_path = SCENE_DIR / "mobile" / f"{car_name}.urdf"
            if not urdf_path.exists():
                print(f"移动小车URDF文件不存在: {urdf_path}")
                return False
            
            urdf = URDF.load(str(urdf_path))
            self.add_urdf(urdf, "mobile_car", on_slider_change=None)
            print(f"用户 {self.client.client_id} 加载了移动小车: {car_name}")
            return True
        except Exception as e:
            print(f"加载移动小车模型时出错: {e}")
            return False

    def load_object(self, object_name: str):
        print(f"加载对象 {object_name}")
        """加载单个工件对象"""
        if object_name not in self.objects_config:
            return
        
        object_config = self.objects_config[object_name]
        object_path = SCENE_DIR / "objects" / f"{object_name}.ply"
        print(f"对象文件路径: {object_path}")
        if not object_path.exists():
            print(f"对象文件不存在: {object_path}")
            return
        
        mesh = trimesh.load_mesh(str(object_path))

        if "attached_robot" in object_config.keys():
            print(f"对象 {object_name} 附加到机器人 {object_config['attached_robot']}")
        
            attached_robot_name = object_config["attached_robot"]
            
            # 检查机器人是否存在
            if attached_robot_name not in self.robots:
                end_effector_node_name = None
            else:
                frame_name = f"{self.namespace}/robot_frame_{attached_robot_name}"
                root_node_name = f"{frame_name}/{attached_robot_name}"
                urdf = self.robots[attached_robot_name]._urdf
                end_effector_link = urdf.robot.links[-1]
                
                # 使用与 ViserUrdf 相同的逻辑构建末端执行器链接的节点名称
                # 直接使用末端执行器链接的 mesh 节点作为父节点
                if urdf.scene is not None:
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
        else:
            print(f"对象 {object_name} 没有附加到机器人")


        # 将点云添加到viser场景
        position, wxyz = self.get_position(object_name, "object")
        print(" node name: ", end_effector_node_name)
        
        # 根据是否有附加节点来确定点云的名称
        if end_effector_node_name is not None:
            # 附加到末端执行器节点下
            point_cloud_name = f"{end_effector_node_name}/{object_name}"
        else:
            # 独立位置
            point_cloud_name = f"{self.namespace}/{object_name}"
        
        mesh_handle = self.ui.server.scene.add_mesh_simple(
            name=point_cloud_name,
            vertices=mesh.vertices,
            faces=mesh.faces,
            position=position,
            wxyz=wxyz,
        )
        self.objects[object_name] = mesh_handle

    def remove_mobile_car(self, car_name: str):
        """移除指定的移动小车（现在使用URDF方式，统一使用remove_urdf处理）"""
        # 移动小车现在也使用URDF方式，使用remove_urdf统一处理
        self.remove_urdf(car_name, "mobile_car")

    def remove_object(self, object_name: str):
        """移除指定的对象"""
        if object_name not in self.objects:
            return
        self.ui.server.scene.remove_by_name(f"{self.namespace}/{object_name}")
        del self.objects[object_name]

    def load_urdf(self, robot_name: str, on_slider_change: Optional[Callable[[str, np.ndarray], None]] = None) -> bool:
        """加载单个机器人的URDF文件
        
        Args:
            robot_name: 机器人名称（对应URDF文件名，不含.urdf扩展名）
            on_slider_change: slider值变化时的回调函数，参数为(robot_name, joint_config)
                            如果为None，则使用默认的回调函数
        
        Returns:
            bool: 加载成功返回True，失败返回False
        """
        try:
            # 构建URDF文件路径
            urdf_path = SCENE_DIR / "urdf" / f"{robot_name}.urdf"
            
            # 检查文件是否存在
            if not urdf_path.exists():
                print(f"URDF文件不存在: {urdf_path}")
                return False
            
            # 加载URDF文件
            urdf = URDF.load(str(urdf_path))
            
            # 如果没有提供回调函数，使用默认的回调函数（使用统一的可视化更新方法）
            if on_slider_change is None:
                def default_on_slider_change(robot_name: str, joint_config: np.ndarray):
                    """默认的slider变化回调函数"""
                    try:
                        # 使用统一的可视化更新方法
                        self.update_robot_visualization(
                            robot_name,
                            joint_config,
                            update_sliders=False,  # slider已经更新了，不需要再次更新
                            update_end_effector_state=True  # 更新末端执行器状态
                        )
                        print(f"用户 {self.client.client_id} 的机器人 {robot_name} 关节配置已更新")
                    except Exception as e:
                        print(f"更新机器人关节配置时出错: {e}")
                on_slider_change = default_on_slider_change
            
            # 添加URDF到场景
            self.add_urdf(urdf, "robot", on_slider_change=on_slider_change)
            print(f"用户 {self.client.client_id} 加载了机器人: {robot_name}")
            return True
            
        except Exception as e:
            print(f"加载机器人 {robot_name} 的URDF时出错: {e}")
            return False

    def load_scene(self):
        """加载场景：从配置文件读取并加载所有机器人
        
        这个方法封装了场景加载的完整逻辑，包括：
        - 读取配置文件
        - 定义slider变化回调函数
        - 加载所有机器人的URDF
        - 更新按钮状态
        """
        if self.is_scene_loaded:
            return  # 如果场景已经加载，不允许再次加载
        
        try:
            # 先对小车模型进行加载
            for car_name in self.mobile_car_config.keys():
                self.load_mobile_car(car_name)

            # 定义slider变化回调函数（内部实现，使用统一的可视化更新方法）
            def on_slider_change(robot_name: str, joint_config: np.ndarray):
                """当slider值变化时, 更新URDF的关节配置"""
                try:
                    # 使用统一的可视化更新方法
                    self.update_robot_visualization(
                        robot_name,
                        joint_config,
                        update_sliders=False,  # slider已经更新了，不需要再次更新
                        update_end_effector_state=True  # 更新末端执行器状态
                    )
                    # print(f"用户 {self.client.client_id} 的机器人 {robot_name} 关节配置已更新")
                except Exception as e:
                    print(f"更新机器人关节配置时出错: {e}")

            # 加载配置中的所有机器人
            for robot_name in self.robot_config.keys():
                self.load_urdf(robot_name, on_slider_change=on_slider_change)

            # 最后加载工件对象
            for object_name in self.objects_config.keys():
                self.load_object(object_name)
            
            # 标记场景已加载
            self.is_scene_loaded = True
            
            # 禁用加载场景按钮，启用清除场景按钮
            if self.btn_load_scene is not None:
                self.btn_load_scene.disabled = True
            if self.btn_clear_scene is not None:
                self.btn_clear_scene.disabled = False
            
            print(f"用户 {self.client.client_id} 场景加载成功")
        except Exception as e:
            print(f"加载场景时出错: {e}")

    def clear_scene(self):
        """清除该用户目前创建的全部的场景 (机器人、移动小车、对象)"""
        # 移除所有机器人
        robot_names = list(self.robots.keys())
        for robot_name in robot_names:
            self.remove_urdf(robot_name, "robot")

        # 移除所有移动小车
        car_names = list(self.mobile_cars.keys())
        for car_name in car_names:
            self.remove_urdf(car_name, "mobile_car")
        
        # 移除所有对象
        for object_name in self.objects_config.keys():
            self.remove_object(object_name)

        self.is_scene_loaded = False
        # 更新按钮状态
        if self.btn_load_scene is not None:
            self.btn_load_scene.disabled = False
        if self.btn_clear_scene is not None:
            self.btn_clear_scene.disabled = True
        print(f"用户 {self.client.client_id} 的场景已清除")

    def cleanup(self):
        """清理该用户的所有资源（用户断开时调用）"""
        self.clear_scene()
        self.remove_scene_buttons()     # 移除该用户的场景按钮
        
        # 移除仿真进度条
        if self.abb_simulation_slider is not None:
            try:
                self.abb_simulation_slider.remove()
            except Exception as e:
                print(f"删除仿真进度条时出错: {e}")
            self.abb_simulation_slider = None
        
        # 清理轨迹数据
        self.abb_trajectory = None
        self.abb_robot_name = None
        
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
                    
                    # 使用统一的可视化更新方法
                    self.update_robot_visualization(
                        robot_name,
                        joint_config,
                        update_sliders=False,  # slider已经更新了，不需要再次更新
                        update_end_effector_state=True  # 更新末端执行器状态
                    )
                    
                    # 调用外部回调函数（如果提供）
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

            # 初始化规划器
            if robot_name not in self.planners:
                try:
                    self.planners[robot_name] = Planner(robot_name)
                    print(f"成功初始化规划器: {robot_name}")
                except Exception as e:
                    print(f"初始化规划器失败: {robot_name}, 错误: {e}")
                    return
            planner = self.planners[robot_name]

            # 构建末端执行器链接对应的场景节点名称
            # ViserUrdf 使用 {root_node_name}/visual/{link_path} 格式来创建 mesh 节点
            root_node_name = self.robot_frames[robot_name].name
            print(f"root_node_name: {root_node_name}")
            
            # 使用与 ViserUrdf 相同的逻辑构建末端执行器链接的节点名称
            # 直接使用末端执行器链接的 mesh 节点作为父节点
            if urdf.scene is not None:
                # 使用 ViserUrdf 的内部函数来构建节点名称
                prefixed_root = f"{root_node_name}/{robot_name}/visual"
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
                        
                        # 使用统一的可视化更新方法
                        self.update_robot_visualization(
                            robot_name,
                            new_joint_config,
                            update_sliders=False,  # 拖动末端执行器时，同步更新slider
                            update_end_effector_state=True  # 更新末端执行器状态
                        )
                        
                        # print(f"用户 {self.client.client_id} 的机器人 {robot_name} 逆解成功")
                    else:
                        pass
                        # print(f"用户 {self.client.client_id} 的机器人 {robot_name} 逆解失败，无解")
                        
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


    def abb_offline_planning(self):
        """ABB框架移动离线规划
        
        规划完成后，保存轨迹并创建仿真进度条
        """
        robot_name = "abb_irb6700_150_320"
        if robot_name not in self.planners:
            print(f"警告: 机器人 {robot_name} 规划器未加载，无法进行规划")
            return
        
        if robot_name not in self.robots:
            print(f"警告: 机器人 {robot_name} 未加载，无法进行规划")
            return
        
        planner = self.planners[robot_name]
        q_start = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        q_end = np.array([0, 0.35, 0.67, 0, -1.02, 0])
        trajectory = planner.sample_trajectory(q_start, q_end, n=50, include_start=True, inclue_end=True)
        
        # 保存轨迹
        self.abb_trajectory = trajectory
        self.abb_robot_name = robot_name
        
        # 如果进度条已存在，先移除
        if self.abb_simulation_slider is not None:
            try:
                self.abb_simulation_slider.remove()
            except Exception as e:
                print(f"移除旧进度条时出错: {e}")
        
        # 在"查看仿真"文件夹下创建进度条
        with self.ui.abb_view_simulation:
            slider_name = f"用户{self.client.client_id}_轨迹"
            self.abb_simulation_slider = self.ui.server.gui.add_slider(
                slider_name,
                min=0.0,
                max=len(self.abb_trajectory) - 1,  # 最大值为轨迹长度减1（因为索引从0开始）
                step=1,
                initial_value=0,
            )
            
            # 为进度条设置事件处理（槽函数）
            @self.abb_simulation_slider.on_update
            def on_slider_update(event: viser.GuiEvent[viser.GuiSliderHandle]):
                """当进度条值变化时，更新仿真状态"""
                step_index = int(event.target.value)  # 进度条的值直接对应轨迹索引
                num_steps = len(self.abb_trajectory)
                step_index = max(0, min(step_index, num_steps - 1))
                joint_config = self.abb_trajectory[step_index]
                self.update_robot_visualization(
                    self.abb_robot_name, 
                    joint_config,
                    update_sliders=True,
                    update_end_effector_state=True
                )
        print(f"已为用户 {self.client.client_id} 创建仿真进度条，轨迹长度: {len(self.abb_trajectory)}")
    
    def update_robot_visualization(self, entity_name: str, joint_config: np.ndarray, update_sliders: bool = True, update_end_effector_state: bool = True):
        """统一的可视化更新回调函数，兼容所有情况（机器人和移动小车）
        
        这个方法可以被以下情况调用：
        1. 拖动关节slider
        2. 拖动末端执行器（通过IK求解后，仅机器人）
        3. 拖动轨迹进度条
        
        Args:
            entity_name: 实体名称（机器人或小车名称）
            joint_config: 关节配置数组，形状为 (dof,)
            update_sliders: 是否同步更新关节slider的值（默认True）
            update_end_effector_state: 是否更新末端执行器控件的状态（默认True，仅机器人有效）
        """
        # 自动检测实体类型（机器人或移动小车）
        is_robot = entity_name in self.robots
        is_mobile_car = entity_name in self.mobile_cars
        
        if not is_robot and not is_mobile_car:
            return  # 实体不存在
        
        # 确保joint_config是numpy数组
        joint_config = np.asarray(joint_config, dtype=np.float64)
        
        # 从对应的字典中获取可视化句柄
        if is_robot:
            viser_urdf_handle = self.robots[entity_name]
            entity_type = "robot"
        else:
            viser_urdf_handle = self.mobile_cars[entity_name]
            entity_type = "mobile_car"
        
        # 更新实体可视化
        viser_urdf_handle.update_cfg(joint_config)
        
        # 同步更新关节slider的值（如果存在且需要更新）
        if update_sliders and entity_name in self.robot_sliders:
            robot_info = self.robot_sliders[entity_name]
            sliders = robot_info["sliders"]
            actuated_joints = robot_info["actuated_joints"]
            
            for i, joint_name in enumerate(actuated_joints):
                if joint_name in sliders and i < len(joint_config):
                    sliders[joint_name].value = float(joint_config[i])
        
        # 更新末端执行器控件的当前关节配置状态（仅机器人，如果存在且需要更新）
        if update_end_effector_state and entity_type == "robot" and entity_name in self.end_effector_controls:
            robot_info = self.end_effector_controls[entity_name]
            robot_info["current_joint_config"] = joint_config.copy()