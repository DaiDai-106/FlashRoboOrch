from numpy._core.multiarray import scalar
import viser
import time
import json
import numpy as np
import ampl
import trimesh
import cv2 as cv
import requests
import re
import math
from pathlib import Path
from typing import Dict, Any, Optional, Callable
from robots_orchestra.viz.viser import ViserUI
from viser.extras import ViserUrdf
from yourdfpy import URDF
from robots_orchestra.planner.ik_solver import IKSolver
from robots_orchestra.planner.planner import Planner
from robots_orchestra import SCENE_DIR
from robots_orchestra import CAMERA_CACHE_DIR
from viser.extras._urdf import _viser_name_from_frame
from robots_orchestra.controller.utils import download_camera_images
from robots_orchestra.controller.utils import depth2pcd
from robots_orchestra.driver.marvin.robot import DCSS

# 用户会话，负责管理每个用户的所有私有资源
class UserSession:
    def __init__(self, client: viser.ClientHandle, viser_ui: ViserUI):
        self.ui = viser_ui
        self.client = client
        self.marvin_kine = None
        self.marvin_kine_2 = None

        self.is_scene_loaded = False # 场景加载状态标志

        self.namespace = f"/world/user_{client.client_id}" # 用户特定的命名空间前缀
        self.robots: Dict[str, Any] = {} # 存储该用户的所有机器人URDF可视化对象
        self.mobile_cars: Dict[str, Any] = {} # 存储该用户的所有移动小车可视化对象
        self.objects: Dict[str, Any] = {} # 存储该用户的所有对象可视化对象

        self.robot_frames: Dict[str, Any] = {} # 存储该用户每个机器人的基座base frame（用于设置位置）
        self.mobile_car_frames: Dict[str, Any] = {} # 存储该用户每个移动小车的基座base frame（用于设置位置）
        self.robot_sliders: Dict[str, Dict[str, Any]] = {} # 存储该用户每个机器人的关节slider控件, 结构: {robot_name: {folder, sliders, actuated_joints}}
        self.end_effector_controls: Dict[str, Dict[str, Any]] = {} # 存储该用户每个机器人的末端执行器轨道工具, 结构: {robot_name: {controls, controls_name, ik_solver, current_joint_config}}
        self.mobile_car_controls: Dict[str, Dict[str, Any]] = {} # 存储该用户每个移动小车的base轨道工具, 结构: {car_name: {controls, controls_name, frame}}
        self.ik_solvers: Dict[str, IKSolver] = {} # 存储该用户的IK求解器（按机器人名称）

        self.robot_config = self.load_robot_config() # 加载机器人位置配置
        self.mobile_car_config = self.load_mobile_car_config() # 加载移动小车位置配置
        self.objects_config = self.load_objects_config() # 加载对象位置配置
    
        # 初始化部分功能
        self.btn_load_scene: Optional[Any] = None # 存储该用户的场景加载按钮控件
        self.btn_clear_scene: Optional[Any] = None # 存储该用户的场景清除按钮控件
        self.marvin_crawl_pose = None
        self.marvin_first_put_pose = None
        # TODO 每一个用户加载时, 还是可以和服务进行共用的

        # ------------------------ 1. 第一个的装配任务 -------------------------------------------------------
        self.first_assem_zhua_pose = None  # 这都是通过icp匹配后的结果
        self.first_assem_put_pose = None
        self.pre_zhua_to_pre_put_movel_path = None
        self.zhua_in_path = None
        self.put_in_path = None

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
        
        # 移除base轨道工具（仅移动小车有）
        if entity_name in self.mobile_car_controls:
            self.remove_base_orbit_tool(entity_name)
        
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

    def load_scene(self, marvin_kine, marvin_kine_2):
        """加载场景：从配置文件读取并加载所有机器人
        
        这个方法封装了场景加载的完整逻辑，包括：
        - 读取配置文件
        - 定义slider变化回调函数
        - 加载所有机器人的URDF
        - 更新按钮状态
        """

        self.marvin_kine = marvin_kine
        self.marvin_kine_2 = marvin_kine_2
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
                robot_folder = self.ui.server.gui.add_folder(robot_name, expand_by_default = False)
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
            
            urdf.update_cfg(joint_config)

            # 构建末端执行器链接对应的场景节点名
            root_node_name = self.robot_frames[robot_name].name
            print(f"root_node_name: {root_node_name}")
            
            # 将轨道工具添加到末端执行器链接的mesh节点下（作为子节点）
            controls_name = f"{root_node_name}/orbit_controls"
            
            fk_result = None
            if robot_name == "left_tianji" and self.marvin_kine is not None:
                    # 左臂使用 marvin_kine，robot_serial=0
                    # 将关节角度从弧度转换为度（marvin 正解器需要度）
                    joints_deg = np.degrees(joint_config).tolist()
                    fk_mat = self.marvin_kine.fk(robot_serial=0, joints=joints_deg)
                    if fk_mat and isinstance(fk_mat, list):
                        fk_result = np.array(fk_mat, dtype=np.float64)
                    else:
                        print(f"警告: marvin 正解失败，robot_name={robot_name}")
                        return
            elif robot_name == "right_tianji" and self.marvin_kine_2 is not None:
                # 右臂使用 marvin_kine_2，robot_serial=1
                # 将关节角度从弧度转换为度（marvin 正解器需要度）
                joints_deg = np.degrees(joint_config).tolist()
                fk_mat = self.marvin_kine_2.fk(robot_serial=1, joints=joints_deg)
                if fk_mat and isinstance(fk_mat, list):
                    fk_result = np.array(fk_mat, dtype=np.float64)

            # 提取位置和四元数
            position = fk_result[:3, 3]
            position = position * 0.001  # 将位置从毫米转换为米
            
            qt7 = ampl.tf44_to_qt7(fk_result)
            wxyz = (float(qt7[3]), float(qt7[0]), float(qt7[1]), float(qt7[2]))

            # 使用单位变换（轨道工具就在末端执行器位置）
            controls_handle = self.ui.server.scene.add_transform_controls(
                name=controls_name,
                position=position,
                wxyz=wxyz,
                scale=0.3,
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
                "current_joint_config": joint_config.copy(),
            }
            
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

    def remove_base_orbit_tool(self, car_name: str):
        """移除指定移动小车的base轨道工具"""
        if car_name not in self.mobile_car_controls:
            return
        
        # 移除场景中的控件（通过删除场景节点）
        car_info = self.mobile_car_controls[car_name]
        controls_name = car_info["controls_name"]
        try:
            # 尝试通过场景API删除
            self.ui.server.scene.remove_by_name(controls_name)
        except Exception as e:
            print(f"删除移动小车base控件时出错: {e}")
        
        del self.mobile_car_controls[car_name]

    def left_marvin_grip(self, marvin_driver):
        """Marvin左臂夹抓"""
        print(f"已为用户 {self.client.client_id} 执行Marvin左臂夹抓")
        result = marvin_driver.set_gripper_force('A', 100, channel=2)
        result = marvin_driver.set_gripper_position('A', 0, channel=2)
        print(f"已为用户 {self.client.client_id} 已经夹抓")

    def left_marvin_release(self, marvin_driver):
        """Marvin左臂释放"""
        print(f"已为用户 {self.client.client_id} 执行Marvin左臂释放")
        result = marvin_driver.set_gripper_force('A', 0, channel=2)
        result = marvin_driver.set_gripper_position('A', 1000, channel=2)
        print(f"已为用户 {self.client.client_id} 已经释放")

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
        
        # 更新末端执行器控件的状态（仅机器人且需要更新时）
        if update_end_effector_state and is_robot and entity_name in self.end_effector_controls:
            robot_info = self.end_effector_controls[entity_name]
            controls_handle = robot_info["controls"]
            
            try:
                # 判断是否是 marvin 机器人，使用 marvin 正解器
                if entity_name == "left_tianji" and self.marvin_kine is not None:
                    # 左臂使用 marvin_kine，robot_serial=0
                    # 将关节角度从弧度转换为度（marvin 正解器需要度）
                    joints_deg = np.degrees(joint_config).tolist()
                    fk_mat = self.marvin_kine.fk(robot_serial=0, joints=joints_deg)
                    if fk_mat and isinstance(fk_mat, list):
                        fk_result = np.array(fk_mat, dtype=np.float64)
                    else:
                        print(f"警告: marvin 正解失败，entity_name={entity_name}")
                        return
                elif entity_name == "right_tianji" and self.marvin_kine_2 is not None:
                    # 右臂使用 marvin_kine_2，robot_serial=1
                    # 将关节角度从弧度转换为度（marvin 正解器需要度）
                    joints_deg = np.degrees(joint_config).tolist()
                    fk_mat = self.marvin_kine_2.fk(robot_serial=1, joints=joints_deg)
                    if fk_mat and isinstance(fk_mat, list):
                        fk_result = np.array(fk_mat, dtype=np.float64)
                    else:
                        print(f"警告: marvin 正解失败，entity_name={entity_name}")
                        return
                
                # 提取位置和四元数
                position = fk_result[:3, 3]
                position = position * 0.001  # 将位置从毫米转换为米
                # 转换为四元数 (w, x, y, z)
                qt7 = ampl.tf44_to_qt7(fk_result)
                wxyz = (float(qt7[3]), float(qt7[0]), float(qt7[1]), float(qt7[2]))
                
                # 更新控件的位置和姿态
                controls_handle.position = tuple(position)
                controls_handle.wxyz = wxyz
                
                # 更新存储的当前关节配置
                robot_info["current_joint_config"] = joint_config.copy()
            except Exception as e:
                print(f"更新末端执行器状态时出错: {e}")
                import traceback
                traceback.print_exc()

        # 

    def get_marvin_joint_data(self, sub_data, arm_index):
        if arm_index < len(sub_data["outputs"]):
            output = sub_data["outputs"][arm_index]
            data_map = {
                "pos": output.get("fb_joint_pos", [0.0] * 7),
                "vel": output.get("fb_joint_vel", [0.0] * 7),
                "sToq": output.get("fb_joint_sToq", [0.0] * 7),
                "cToq": output.get("fb_joint_cToq", [0.0] * 7),
                "them": output.get("fb_joint_them", [0.0] * 7),
            }
            return data_map.get("pos", [0.0] * 7)
        else:
            return [0.0] * 7

    #----------------------------------------------------------天机装配任务处理----------------------------------------------------------
    def get_icp_transformation(
        self,
        pcd_url: str,
        mesh_url: str,
        task_id: str = "marvin",
        skip_points: int = 2,
        max_dist_point_to_plane: float = 0.1,
        scale: float = 0.001,
        subtask: str = "",
        api_url: str = "http://192.168.1.206:8001/icp_pc_to_mesh",
        timeout: int = 30
    ) -> Optional[np.ndarray]:
        """
        调用ICP配准服务获取变换矩阵
        
        Args:
            pcd_url: 点云文件的URL
            mesh_url: 网格文件的URL
            task_id: 任务ID，默认为"marvin"
            skip_points: 跳过的点数
            max_dist_point_to_plane: 点到平面的最大距离
            scale: 缩放比例
            subtask: 子任务名称
            api_url: ICP服务API地址
            timeout: 请求超时时间（秒）
            
        Returns:
            4x4变换矩阵 (np.ndarray)，如果调用失败则返回None
        """
        request_data = {
            "task_id": task_id,
            "input": {
                "pcd_url": pcd_url,
                "mesh_url": mesh_url
            },
            "settings": {
                "skip_points": skip_points,
                "max_dist_point_to_plane": max_dist_point_to_plane,
                "scale": scale,
                "subtask": subtask
            }
        }
        
        try:
            response = requests.post(api_url, json=request_data, timeout=timeout)
            response.raise_for_status()  # 如果状态码不是200会抛出异常
            result = response.json()
            print(f"ICP服务响应: {result}")
            
            if "output" in result and "tf_src_tgt" in result["output"]:
                transformation_matrix = np.array(result["output"]["tf_src_tgt"])
                print(f"成功从服务获取变换矩阵")
                return transformation_matrix
            else:
                # 如果响应格式不同，可能需要手动解析
                print(f"警告: 未找到预期的矩阵字段，响应内容: {result}")
                return None
        except requests.exceptions.RequestException as e:
            print(f"ICP服务调用失败: {e}")
            return None

    def get_frame_transform_matrix(self, frame) -> np.ndarray:
        """
        从 viser frame 对象获取其变换矩阵
        
        Args:
            frame: viser 的 frame 对象（有 position 和 wxyz 属性）
            
        Returns:
            4x4 变换矩阵 (np.ndarray)
        """
        frame_position = np.array(frame.position, dtype=np.float64)  # (x, y, z)
        frame_wxyz = np.array(frame.wxyz, dtype=np.float64)  # (w, x, y, z)
        
        # 构建 qt7 格式: [qx, qy, qz, qw, x, y, z]
        qt7_frame = np.array([
            frame_wxyz[1],  # qx
            frame_wxyz[2],  # qy
            frame_wxyz[3],  # qz
            frame_wxyz[0],  # qw
            frame_position[0],  # x
            frame_position[1],  # y
            frame_position[2]   # z
        ], dtype=np.float64)
        
        # 转换为 4x4 变换矩阵
        return ampl.qt7_to_tf44(qt7_frame)

    def rotation_matrix_from_axis_angle(self, axis, angle):
        """
        Rodrigues 公式：从轴角表示计算旋转矩阵
        
        Args:
            axis: 旋转轴向量 (3,)
            angle: 旋转角度（弧度）
            
        Returns:
            3x3 旋转矩阵
        """
        normalize = lambda v: v / np.linalg.norm(v)
        axis = normalize(axis)
        x, y, z = axis
        c = np.cos(angle)
        s = np.sin(angle)
        C = 1 - c

        return np.array([
            [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
            [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
            [z*x*C - y*s,   z*y*C + x*s, c + z*z*C]
        ])

    def align_model_transform_in_base(self, T_ref, T_new):
        """先只绕t_now的x轴旋转,对其上表面与t_ref的z轴对齐"""
        R_ref = T_ref[:3, :3]
        R_new = T_new[:3, :3]

        normalize = lambda v: v / np.linalg.norm(v)

        z_ref = normalize(R_ref[:, 2])
        z_new = normalize(R_new[:, 2])
        x_new = normalize(R_new[:, 0])

        project_to_plane = lambda v, n: v - np.dot(v, n) * n
        z_ref_p = project_to_plane(z_ref, x_new)
        z_new_p = project_to_plane(z_new, x_new)

        z_ref_p = normalize(z_ref_p)
        z_new_p = normalize(z_new_p)

        # 有符号角度（方向由 x_new 决定）
        cross = np.cross(z_new_p, z_ref_p)
        sin_angle = np.dot(cross, x_new)
        cos_angle = np.dot(z_new_p, z_ref_p)
        angle = np.arctan2(sin_angle, cos_angle)

         # 绕自身 X 轴旋转
        R_corr = self.rotation_matrix_from_axis_angle(x_new, angle)

        R_aligned = R_corr @ R_new

        # ===== 第二阶段：检查 X 轴是否反向 =====
        x_ref = normalize(R_ref[:, 0])
        x_cur = normalize(R_aligned[:, 0])

        if np.dot(x_cur, x_ref) < 0:
            # 绕当前 Z 轴旋转 180°
            z_cur = normalize(R_aligned[:, 2])
            R_flip = self.rotation_matrix_from_axis_angle(z_cur, np.pi)
            R_aligned = R_flip @ R_aligned

        T_out = T_new.copy()
        T_out[:3, :3] = R_aligned
        return T_out

    
    def right_marvin_home(self, marvin_driver):
        marvin_driver.clear_set()
        marvin_driver.set_state(arm='B',state = 1) # 位置跟随模式
        marvin_driver.set_vel_acc(arm='B',velRatio=5, AccRatio=10)
        marvin_driver.send_cmd()
        time.sleep(0.5)
        
        marvin_driver.clear_set()
        joint_cmd_1=[ 0, 0, 10.18,-56.21, 44.37, 26.81, 0]
        marvin_driver.set_joint_cmd_pose(arm='B',joints=joint_cmd_1)
        marvin_driver.send_cmd()
        time.sleep(3) #预留运动时间

    def right_marvin_first_capture(self, marvin_driver):
        print(f"已为用户 {self.client.client_id} 执行Marvin右臂第一次拍照")
        marvin_driver.clear_set()
        marvin_driver.set_state(arm='B',state = 1) # 位置跟随模式
        marvin_driver.set_vel_acc(arm='B',velRatio=5, AccRatio=10)
        marvin_driver.send_cmd()
        time.sleep(0.5)
        
        marvin_driver.clear_set()
        joint_cmd_1=[ -54.85, 34.56, 45.79, -75.46, 67.91, -1.49, 85.68]
        marvin_driver.set_joint_cmd_pose(arm='B',joints=joint_cmd_1)
        marvin_driver.send_cmd()
        time.sleep(3) #预留运动时间

    def right_marvin_first_icp(self, marvin_driver, marvin_kine):
         # 其实这里的第一步是先获取前端的服务，然后机械臂才会去运动
        MASH_URL = "http://192.168.1.209:8000/api/latestpcdmask"
        response = requests.get(MASH_URL)
        print(f"response: {response.json()}")
        pcd_url = response.json()["pcd_mask_url"]

        ref_raw_locolizaiton = [[ 0.60605383, 0.64962685, -0.45900303, 0.63666517],
                                [ 0.11070016, -0.64032441, -0.7600857, 0.59818721],
                                [-0.78768283, 0.40984124, -0.45998499, 0.14560415],
                                [ 0.0, 0.0, 0.0, 1.0]]

        # 调用ICP配准服务获取变换矩阵
        # TODO 后续这里尽可能的读取实际的xyz abc进来的
        cali_path = Path(__file__).parent / "camera_cache" / "icp" / "handeye_hand.npy"
        cali_matrix = np.load(cali_path)

        print(f'cali_matrix: {cali_matrix}')

        # 调用ICP配准服务获取变换矩阵
        raw_locolizaiton = self.get_icp_transformation(
            pcd_url=pcd_url,
            mesh_url="http://192.168.1.206:9000/storage/dualp/part_04_mm.ply"
        )

        print(f'raw_locolizaiton: {raw_locolizaiton}')
        
        # 如果服务调用失败，使用默认的硬编码值作为后备
        if raw_locolizaiton is None:
            print("使用默认的 raw_locolizaiton 矩阵")
            raw_locolizaiton = None


        raw_locolizaiton = np.array(raw_locolizaiton)
        print(f'raw_locolizaiton: {raw_locolizaiton}')
        
        if raw_locolizaiton is None:
            print("raw_locolizaiton 为空")
            return

        # 计算矩阵的逆
        raw_locolizaiton_inv = np.linalg.inv(raw_locolizaiton)
        print(f'raw_locolizaiton_inv: {raw_locolizaiton_inv}')

        camera_pose = marvin_kine.xyzabc_to_mat4x4([212.17 * 0.001, 87.04 * 0.001, 572.86 * 0.001, 126.67, 40.79, 58.42])

        last_raw_locolizaiton =  camera_pose @ cali_matrix @ raw_locolizaiton_inv
        last_camera_pose =  camera_pose @ cali_matrix


        robot_frame = self.robot_frames["right_tianji"]
        frame_name = robot_frame.name
        
        # 获取 robot_frame 的变换矩阵
        frame_transform_matrix = self.get_frame_transform_matrix(robot_frame)
        print(f"robot_frame 变换矩阵:\n{frame_transform_matrix}")

        model_in_base_ref = frame_transform_matrix @ camera_pose @ cali_matrix @ np.linalg.inv(ref_raw_locolizaiton)
        model_in_base_now = frame_transform_matrix @ last_raw_locolizaiton
        model_in_base_now = self.align_model_transform_in_base(model_in_base_ref, model_in_base_now)
        print(f"model_in_base_now: {model_in_base_now}")
        last_raw_locolizaiton = np.linalg.inv(frame_transform_matrix) @ model_in_base_now


        qt7 = ampl.tf44_to_qt7( np.array(last_raw_locolizaiton))
        position = (float(qt7[4]), float(qt7[5]), float(qt7[6]))
        wxyz = (float(qt7[3]), float(qt7[0]), float(qt7[1]), float(qt7[2]))
        frame_a_name = f"{frame_name}/model_7"
        frame_a = self.ui.server.scene.add_frame(
            name=frame_a_name,
            show_axes=True,  # 显示坐标轴以便调试
            position=position,
            wxyz=wxyz,
        )       

        # 将三角网格添加到frame A下
        ply_path = "/home/daidai/FlashRoboOrch/tests/data/zhuaqu_pcd/part_04_mm.ply"
        vertices, faces = ampl.read_trimesh(ply_path)
        print(f"vertices shape: {vertices.shape}")
        print(f"faces shape: {faces.shape}")
        vertices = vertices * 0.001  # 将每个顶点的位置值乘以 0.001（从毫米转换为米）
            
        mesh_name = f"{frame_a_name}/icp_1"
        self.ui.server.scene.add_mesh_simple(
            name=mesh_name,
            vertices=vertices,
            faces=faces,
        )
        print(f"三角网格已添加到frame A下: {mesh_name}")

        qt7 = ampl.tf44_to_qt7( np.array(last_camera_pose))
        position = (float(qt7[4]), float(qt7[5]), float(qt7[6]))
        wxyz = (float(qt7[3]), float(qt7[0]), float(qt7[1]), float(qt7[2]))
        frame_b_name = f"{frame_name}/model_mask"
        frame_b = self.ui.server.scene.add_frame(
            name=frame_b_name,
            show_axes=False,  # 显示坐标轴以便调试
            position=position,
            wxyz=wxyz,
        )

        ply_path = "/home/daidai/FlashRoboOrch/tests/data/zhuaqu_pcd/pcd_mask_1.ply"
        pcd_v, pcd_c, _ = ampl.read_pointcloud(ply_path)
        pcd_v = pcd_v * 0.001  # 将每个点的位置值乘以 0.001
        mesh_name = f"{frame_b_name}/icp_1"
        self.ui.server.scene.add_point_cloud(
            name=mesh_name,
            points=pcd_v,
            colors=pcd_c,
            point_size=0.01,  # 点的大小
        )
        print(f"点云已添加到frame B下: {mesh_name}")    
    

        # 这个是在urdf下的坐标系下的变换了
        self.first_assem_put_pose = frame_transform_matrix @ last_raw_locolizaiton

    def right_marvin_second_capture(self, marvin_driver):
        print(f"已为用户 {self.client.client_id} 执行Marvin右臂第二次拍照")
        marvin_driver.clear_set()
        marvin_driver.set_state(arm='B',state = 1) # 位置跟随模式
        marvin_driver.set_vel_acc(arm='B',velRatio=5, AccRatio=10)
        marvin_driver.send_cmd()
        time.sleep(0.5)
        
        marvin_driver.clear_set()
        joint_cmd_1=[ 1.19, -82.97, 10.18,-56.21, 44.37, 26.81, 73.96]
        marvin_driver.set_joint_cmd_pose(arm='B',joints=joint_cmd_1)
        marvin_driver.send_cmd()
        time.sleep(3) #预留运动时间

    def right_marvin_second_icp(self, marvin_driver, marvin_kine): 
        # 其实这里的第一步是先获取前端的服务，然后机械臂才会去运动
        MASH_URL = "http://192.168.1.209:8000/api/latestpcdmask"
        response = requests.get(MASH_URL)
        print(f"response: {response.json()}")
        pcd_url = response.json()["pcd_mask_url"]

        ref_raw_locolizaiton = [
            [
                -0.9890800714492798,
                0.1187606081366539,
                -0.0872708335518837,
                0.12945935130119324
            ],
            [
                0.14450779557228088,
                0.6652054190635681,
                -0.7325431108474731,
                0.5571730136871338
            ],
            [
                -0.028944265097379684,
                -0.7371553182601929,
                -0.6751030087471008,
                0.49707046151161194
            ],
            [
                0,
                0,
                0,
                1
            ]
        ]

        # 调用ICP配准服务获取变换矩阵
        # TODO 后续这里尽可能的读取实际的xyz abc进来的
        cali_path = Path(__file__).parent / "camera_cache" / "icp" / "handeye_hand.npy"
        cali_matrix = np.load(cali_path)

        print(f'cali_matrix: {cali_matrix}')

        # 调用ICP配准服务获取变换矩阵
        raw_locolizaiton = self.get_icp_transformation(
            pcd_url=pcd_url,
            mesh_url="http://192.168.1.206:9000/storage/dualp/part_05_mm.ply"
        )
        
        # 如果服务调用失败，使用默认的硬编码值作为后备
        if raw_locolizaiton is None:
            print("使用默认的 raw_locolizaiton 矩阵")
            raw_locolizaiton = ref_raw_locolizaiton


        raw_locolizaiton = np.array(raw_locolizaiton)
        print(f'raw_locolizaiton: {raw_locolizaiton}')
        
        # 计算矩阵的逆
        raw_locolizaiton_inv = np.linalg.inv(raw_locolizaiton)
        print(f'raw_locolizaiton_inv: {raw_locolizaiton_inv}')

        camera_pose = marvin_kine.xyzabc_to_mat4x4([480.57 * 0.001, -5.98 * 0.001, -95.91 * 0.001, -154.49, 39.60, 106.45])

        last_raw_locolizaiton =  camera_pose @ cali_matrix @ raw_locolizaiton_inv
        last_camera_pose =  camera_pose @ cali_matrix


        robot_frame = self.robot_frames["right_tianji"]
        frame_name = robot_frame.name
        
        # 获取 robot_frame 的变换矩阵
        frame_transform_matrix = self.get_frame_transform_matrix(robot_frame)
        print(f"robot_frame 变换矩阵:\n{frame_transform_matrix}")

        model_in_base_ref = frame_transform_matrix @ camera_pose @ cali_matrix @ np.linalg.inv(ref_raw_locolizaiton)
        model_in_base_now = frame_transform_matrix @ last_raw_locolizaiton
        model_in_base_now = self.align_model_transform_in_base(model_in_base_ref, model_in_base_now)
        print(f"model_in_base_now: {model_in_base_now}")
        last_raw_locolizaiton = np.linalg.inv(frame_transform_matrix) @ model_in_base_now


        qt7 = ampl.tf44_to_qt7( np.array(last_raw_locolizaiton))
        position = (float(qt7[4]), float(qt7[5]), float(qt7[6]))
        wxyz = (float(qt7[3]), float(qt7[0]), float(qt7[1]), float(qt7[2]))
        frame_a_name = f"{frame_name}/model_7"
        frame_a = self.ui.server.scene.add_frame(
            name=frame_a_name,
            show_axes=False,  # 显示坐标轴以便调试
            position=position,
            wxyz=wxyz,
        )       

        # 将三角网格添加到frame A下
        ply_path = "/home/daidai/FlashRoboOrch/tests/data/zhuaqu_pcd/part_05_mm.ply"
        vertices, faces = ampl.read_trimesh(ply_path)
        print(f"vertices shape: {vertices.shape}")
        print(f"faces shape: {faces.shape}")
        vertices = vertices * 0.001  # 将每个顶点的位置值乘以 0.001（从毫米转换为米）
            
        mesh_name = f"{frame_a_name}/icp_2"
        self.ui.server.scene.add_mesh_simple(
            name=mesh_name,
            vertices=vertices,
            faces=faces,
        )
        print(f"三角网格已添加到frame A下: {mesh_name}")

        qt7 = ampl.tf44_to_qt7( np.array(last_camera_pose))
        position = (float(qt7[4]), float(qt7[5]), float(qt7[6]))
        wxyz = (float(qt7[3]), float(qt7[0]), float(qt7[1]), float(qt7[2]))
        frame_b_name = f"{frame_name}/model_mask"
        frame_b = self.ui.server.scene.add_frame(
            name=frame_b_name,
            show_axes=False,  # 显示坐标轴以便调试
            position=position,
            wxyz=wxyz,
        )

        ply_path = "/home/daidai/FlashRoboOrch/tests/data/zhuaqu_pcd/pcd_mask.ply"
        pcd_v, pcd_c, _ = ampl.read_pointcloud(ply_path)
        pcd_v = pcd_v * 0.001  # 将每个点的位置值乘以 0.001
        mesh_name = f"{frame_b_name}/icp_2"
        self.ui.server.scene.add_point_cloud(
            name=mesh_name,
            points=pcd_v,
            colors=pcd_c,
            point_size=0.01,  # 点的大小
        )
        print(f"点云已添加到frame B下: {mesh_name}")    
    

        # 这个是在urdf下的坐标系下的变换了
        self.first_assem_zhua_pose = frame_transform_matrix @ last_raw_locolizaiton
 
    def left_marvin_zhua_pose(self, marvin_driver, marvin_kine):
        R = np.array([
            [0, -1, 0],
            [-1, 0, 0],
            [0, 0, -1]
        ])

        t = np.array([0.0, 0.0, 0.26])

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t

        robot_frame = self.robot_frames["left_tianji"]
        frame_name = robot_frame.name
        left_base = self.get_frame_transform_matrix(robot_frame)

        # 后面是可以补偿这个值的
        pre_zhua_pose = [369.54 * 0.001, 212.06 * 0.001, 556.58 * 0.001, 86.56, -58.18, 176.96 ]
        pre_zhua_pose = marvin_kine.xyzabc_to_mat4x4(pre_zhua_pose)

        qt7 = ampl.tf44_to_qt7( np.array(pre_zhua_pose))
        position = (float(qt7[4]), float(qt7[5]), float(qt7[6]))
        wxyz = (float(qt7[3]), float(qt7[0]), float(qt7[1]), float(qt7[2]))
        frame_b_name = f"{frame_name}/pre_zhua_pose"
        frame_pre_zhua_pose = self.ui.server.scene.add_frame(
            name=frame_b_name,
            show_axes=True,  # 显示坐标轴以便调试
            position=position,
            wxyz=wxyz,
        )


        real_zhua_pose = np.linalg.inv(left_base) @ self.first_assem_zhua_pose @ T
        qt7 = ampl.tf44_to_qt7( np.array(real_zhua_pose))
        position = (float(qt7[4]), float(qt7[5]), float(qt7[6]))
        wxyz = (float(qt7[3]), float(qt7[0]), float(qt7[1]), float(qt7[2]))
        frame_c_name = f"{frame_name}/real_zhua_pose"
        frame_real_zhua_pose = self.ui.server.scene.add_frame(
            name=frame_c_name,
            show_axes=True,  # 显示坐标轴以便调试
            position=position,
            wxyz=wxyz,
        )

        self.first_assem_zhua_pose = real_zhua_pose
    
    def left_marvin_zhua_l(self, marvin_driver, marvin_kine):
        ref_joints = [ -18.5, -17.77, 52.43, -61.28, 149.22, -10.41, 56.33]

        end_pose = self.first_assem_zhua_pose
        end_pose[0][3] *= 1000
        end_pose[1][3] *= 1000
        end_pose[2][3] *= 1000

        print(f"end_pose: {end_pose}")

        end_ik = marvin_kine.ik(robot_serial=0,pose_mat=end_pose, ref_joints=ref_joints)
        print(f"end_ik: {end_ik}")
        joint_data = end_ik.m_Output_RetJoint.to_list()
        print(f"joint_data: {joint_data}")

        save_path = Path(__file__).parent / "path_cache" / "left_marvin_zhua_l.txt"
        success = marvin_kine.movL_KeepJ( robot_serial=0, start_joints=ref_joints, end_joints=joint_data, vel=5, save_path=str(save_path) )
        if success:
            print("movel 求解成功")
        else:
            print("movel 求解失败")

        # 解析txt文件，提取每一行的前7个数值（X, Y, Z, A, B, C, U）
        parsed_data = []
        with open(save_path, 'r') as f:
            lines = f.readlines()
            # 跳过第一行（PoinType=9@6557），从第二行开始解析
            for line in lines[1:]:
                line = line.strip()
                if not line:
                    continue
                
                # 使用正则表达式提取 X, Y, Z, A, B, C, U 的值
                pattern = r'X\s+([-+]?\d+\.?\d*)\$Y\s+([-+]?\d+\.?\d*)\$Z\s+([-+]?\d+\.?\d*)\$A\s+([-+]?\d+\.?\d*)\$B\s+([-+]?\d+\.?\d*)\$C\s+([-+]?\d+\.?\d*)\$U\s+([-+]?\d+\.?\d*)'
                match = re.search(pattern, line)
                if match:
                    values = [float(match.group(i)) for i in range(1, 8)]
                    parsed_data.append(values)
        
        print(f"成功解析 {len(parsed_data)} 行数据")
        
        # 对parsed_data进行采样：保留第一个、最后一个，以及每100个取一个
        original_count = len(parsed_data)
        if len(parsed_data) > 0:
            sampled_indices = set()
            # 保留第一个
            sampled_indices.add(0)
            # 保留最后一个
            sampled_indices.add(len(parsed_data) - 1)
            # 每100个取一个
            for i in range(100, len(parsed_data) - 1, 100):
                sampled_indices.add(i)
            
            # 按索引顺序排序并提取采样后的数据
            sampled_indices = sorted(sampled_indices)
            parsed_data = [parsed_data[i] for i in sampled_indices]
            self.zhua_in_path = parsed_data # 这样还没有单位转换
            # 将每个值从角度转换为弧度
            parsed_data = [[math.radians(val) for val in row] for row in parsed_data]
            print(f"采样后保留 {len(parsed_data)} 个数据点（原始数据: {original_count} 行），已转换为弧度")

        with self.ui.left_marvin_zhua_view_simulation:
            slider_name = f"用户{self.client.client_id}_轨迹"
            self.pre_put_moveL_simulation_slider = self.ui.server.gui.add_slider(
                slider_name,
                min=0.0,
                max=len(parsed_data) - 1,  # 最大值为轨迹长度减1（因为索引从0开始）
                step=1,
                initial_value=0,
            )
            
            # 为进度条设置事件处理（槽函数）
            @self.pre_put_moveL_simulation_slider.on_update
            def on_slider_update(event: viser.GuiEvent[viser.GuiSliderHandle]):
                """当进度条值变化时，更新仿真状态"""
                step_index = int(event.target.value)  # 进度条的值直接对应轨迹索引
                num_steps = len(parsed_data)
                step_index = max(0, min(step_index, num_steps - 1))
                joint_config = parsed_data[step_index]
                self.update_robot_visualization(
                    "left_tianji", 
                    joint_config,
                    update_sliders=True,
                    update_end_effector_state=True
                )
        print(f"已为用户 {self.client.client_id} 创建仿真进度条，轨迹长度: {len(parsed_data)}")
        
        # 函数结束后删除临时文件
        if save_path.exists():
            save_path.unlink()
            print(f"已删除临时文件: {save_path}")

    def left_marvin_zhua_in(self, marvin_driver):
        print(f"已为用户 {self.client.client_id} 执行Marvin左臂抓取进近")
        if self.zhua_in_path is None:
            print("抓取进近的movel路径为空")
            return

        for joint_config in self.zhua_in_path:
            marvin_driver.clear_set()
            joint_cmd_1=joint_config
            marvin_driver.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
            marvin_driver.send_cmd()
            time.sleep(0.2)  #预留运动时间

    def left_marvin_zhua_out(self, marvin_driver):
        print(f"已为用户 {self.client.client_id} 执行Marvin左臂抓取退出")
        if self.zhua_in_path is None:
            print("抓取进近的movel路径为空")
            return

        for joint_config in self.zhua_in_path[::-1]:
            marvin_driver.clear_set()
            joint_cmd_1=joint_config
            marvin_driver.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
            marvin_driver.send_cmd()
            time.sleep(0.2)  #预留运动时间

    #----------------------------------------------------------Marvin左臂装配任务处理----------------------------------------------------------
    def left_marvin_home(self, marvin_driver):
        print(f"已为用户 {self.client.client_id} 执行Marvin左臂回到home位置")
        marvin_driver.clear_set()
        marvin_driver.set_state(arm='A',state = 1) # 位置跟随模式
        marvin_driver.set_vel_acc(arm='A',velRatio=5, AccRatio=10)
        marvin_driver.send_cmd()
        time.sleep(0.5)
        
        marvin_driver.clear_set()
        joint_cmd_1=[ -19.464, 11.7307, 55.1657, -30.5945, 141.4534, -10.4661, 64.3169 ]
        marvin_driver.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
        marvin_driver.send_cmd()
        time.sleep(3) #预留运动时间

    def left_marvin_pre_zhua_put_movel(self, marvin_driver, marvin_kine):
        ref_joints = [ -18.5, -17.77, 52.43, -61.28, 149.22, -10.41, 56.33]
        ref_end_joints = [ 1.09, -95.91, 24.81, -45.18, 140.79, -47.54, 60.27]

        save_path = Path(__file__).parent / "path_cache" / "pre_zhua_put_movel.txt"
        success = marvin_kine.movL_KeepJ( robot_serial=0, start_joints=ref_joints, end_joints=ref_end_joints, vel=20, save_path=str(save_path) )
        if success:
            print("movel 求解成功")
        else:
            print("movel 求解失败")

        # 解析txt文件，提取每一行的前7个数值（X, Y, Z, A, B, C, U）
        parsed_data = []
        with open(save_path, 'r') as f:
            lines = f.readlines()
            # 跳过第一行（PoinType=9@6557），从第二行开始解析
            for line in lines[1:]:
                line = line.strip()
                if not line:
                    continue
                
                # 使用正则表达式提取 X, Y, Z, A, B, C, U 的值
                pattern = r'X\s+([-+]?\d+\.?\d*)\$Y\s+([-+]?\d+\.?\d*)\$Z\s+([-+]?\d+\.?\d*)\$A\s+([-+]?\d+\.?\d*)\$B\s+([-+]?\d+\.?\d*)\$C\s+([-+]?\d+\.?\d*)\$U\s+([-+]?\d+\.?\d*)'
                match = re.search(pattern, line)
                if match:
                    values = [float(match.group(i)) for i in range(1, 8)]
                    parsed_data.append(values)
        
        print(f"成功解析 {len(parsed_data)} 行数据")
        self.pre_zhua_to_pre_put_movel_path = parsed_data # 这样还没有单位转换
        
        # 对parsed_data进行采样：保留第一个、最后一个，以及每100个取一个
        original_count = len(parsed_data)
        if len(parsed_data) > 0:
            sampled_indices = set()
            # 保留第一个
            sampled_indices.add(0)
            # 保留最后一个
            sampled_indices.add(len(parsed_data) - 1)
            # 每100个取一个
            for i in range(100, len(parsed_data) - 1, 100):
                sampled_indices.add(i)
            
            # 按索引顺序排序并提取采样后的数据
            sampled_indices = sorted(sampled_indices)
            parsed_data = [parsed_data[i] for i in sampled_indices]
            # print(f"pre_zhua_to_pre_put_movel_path: {self.pre_zhua_to_pre_put_movel_path[-1]}")
            # 将每个值从角度转换为弧度
            parsed_data = [[math.radians(val) for val in row] for row in parsed_data]
            print(f"采样后保留 {len(parsed_data)} 个数据点（原始数据: {original_count} 行），已转换为弧度")

        with self.ui.left_marvin_pre_zhua_put_movel_view_simulation:
            slider_name = f"用户{self.client.client_id}_轨迹"
            self.pre_put_moveL_simulation_slider = self.ui.server.gui.add_slider(
                slider_name,
                min=0.0,
                max=len(parsed_data) - 1,  # 最大值为轨迹长度减1（因为索引从0开始）
                step=1,
                initial_value=0,
            )
            
            # 为进度条设置事件处理（槽函数）
            @self.pre_put_moveL_simulation_slider.on_update
            def on_slider_update(event: viser.GuiEvent[viser.GuiSliderHandle]):
                """当进度条值变化时，更新仿真状态"""
                step_index = int(event.target.value)  # 进度条的值直接对应轨迹索引
                num_steps = len(parsed_data)
                step_index = max(0, min(step_index, num_steps - 1))
                joint_config = parsed_data[step_index]
                self.update_robot_visualization(
                    "left_tianji", 
                    joint_config,
                    update_sliders=True,
                    update_end_effector_state=True
                )
        print(f"已为用户 {self.client.client_id} 创建仿真进度条，轨迹长度: {len(parsed_data)}")
        
        # 函数结束后删除临时文件
        if save_path.exists():
            save_path.unlink()
            print(f"已删除临时文件: {save_path}")

    def left_marvin_pre_zhua_to_put(self, marvin_driver):
        marvin_driver.clear_set()
        marvin_driver.set_state(arm='A',state = 1) # 位置跟随模式
        marvin_driver.set_vel_acc(arm='A',velRatio=20, AccRatio=10)
        marvin_driver.send_cmd()
        time.sleep(0.5)
        print(f"已为用户 {self.client.client_id} 实机执行Marvin左臂预抓取到预装配位置")
        if self.pre_zhua_to_pre_put_movel_path is None:
            print("预抓取到预装配的movel路径为空")
            return

        for joint_config in self.pre_zhua_to_pre_put_movel_path:
            marvin_driver.clear_set()
            joint_cmd_1=joint_config
            marvin_driver.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
            marvin_driver.send_cmd()
            time.sleep(0.02)  #预留运动时间

    def left_marvin_pre_zhua_to_put_reverse(self, marvin_driver):
        marvin_driver.clear_set()
        marvin_driver.set_state(arm='A',state = 1) # 位置跟随模式
        marvin_driver.set_vel_acc(arm='A',velRatio=20, AccRatio=10)
        marvin_driver.send_cmd()
        time.sleep(0.5)

        print(f"已为用户 {self.client.client_id} 实机执行Marvin左臂回退到预抓取位置")
        if self.pre_zhua_to_pre_put_movel_path is None:
            print("预抓取到预装配的movel路径为空")
            return

        for joint_config in self.pre_zhua_to_pre_put_movel_path[::-1]:
            marvin_driver.clear_set()
            joint_cmd_1=joint_config
            marvin_driver.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
            marvin_driver.send_cmd()
            time.sleep(0.02)  #预留运动时间

    # 左臂从home 过度到 预抓取位置 
    def left_marvin_pre_zhua_move(self, marvin_driver):
        print(f"已为用户 {self.client.client_id} 执行Marvin左臂到预抓取位置")
        marvin_driver.clear_set()
        marvin_driver.set_state(arm='A',state = 1) # 位置跟随模式
        marvin_driver.set_vel_acc(arm='A',velRatio=5, AccRatio=10)
        marvin_driver.send_cmd()
        time.sleep(0.5)
        
        marvin_driver.clear_set()
        joint_cmd_1=[ -18.50, -17.77, 52.43, -61.28, 149.22, -10.41, 56.33]
        marvin_driver.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
        marvin_driver.send_cmd()
        time.sleep(3) #预留运动时间

    # 左臂放置的姿态
    def left_marvin_put_pose(self, marvin_driver, marvin_kine):
        R = np.array([
            [-1, 0, 0],
            [0, 0, -1],
            [0, -1, 0]
        ])

        t = np.array([-0.081, 0.3, -0.145])

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t

        robot_frame = self.robot_frames["left_tianji"]
        frame_name = robot_frame.name
        left_base = self.get_frame_transform_matrix(robot_frame)

        # 后面是可以补偿这个值的
        pre_put_pose = [497.75 * 0.001, 200.11 * 0.001, -86.07 * 0.001, 102.95, -87.24, 165.47 ]
        pre_put_pose = marvin_kine.xyzabc_to_mat4x4(pre_put_pose)

        qt7 = ampl.tf44_to_qt7( np.array(pre_put_pose))
        position = (float(qt7[4]), float(qt7[5]), float(qt7[6]))
        wxyz = (float(qt7[3]), float(qt7[0]), float(qt7[1]), float(qt7[2]))
        frame_b_name = f"{frame_name}/put_pose"
        frame_pre_zhua_pose = self.ui.server.scene.add_frame(
            name=frame_b_name,
            show_axes=True,  # 显示坐标轴以便调试
            position=position,
            wxyz=wxyz,
        )


        real_put_pose = np.linalg.inv(left_base) @ self.first_assem_put_pose @ T
        qt7 = ampl.tf44_to_qt7( np.array(real_put_pose))
        position = (float(qt7[4]), float(qt7[5]), float(qt7[6]))
        wxyz = (float(qt7[3]), float(qt7[0]), float(qt7[1]), float(qt7[2]))
        frame_c_name = f"{frame_name}/real_put_pose"
        frame_real_put_pose = self.ui.server.scene.add_frame(
            name=frame_c_name,
            show_axes=True,  # 显示坐标轴以便调试
            position=position,
            wxyz=wxyz,
        )

        self.first_assem_put_pose = real_put_pose

    # 左臂放置的movel仿真
    def left_marvin_put_l(self, marvin_driver, marvin_kine):
        ref_joints = [-14.758271, -118.672726, 87.272366, -45.18, 68.178012, -23.359094, 52.670127]

        end_pose = self.first_assem_put_pose
        end_pose[0][3] *= 1000
        end_pose[1][3] *= 1000
        end_pose[2][3] *= 1000

        print(f"end_pose: {end_pose}")

        end_ik = marvin_kine.ik(robot_serial=0,pose_mat=end_pose, ref_joints=ref_joints)
        print(f"end_ik: {end_ik}")
        joint_data = end_ik.m_Output_RetJoint.to_list()
        print(f"joint_data: {joint_data}")

        save_path = Path(__file__).parent / "path_cache" / "left_marvin_put_l.txt"
        success = marvin_kine.movL_KeepJ( robot_serial=0, start_joints=ref_joints, end_joints=joint_data, vel=5, save_path=str(save_path) )
        if success:
            print("movel 求解成功")
        else:
            print("movel 求解失败")

        # 解析txt文件，提取每一行的前7个数值（X, Y, Z, A, B, C, U）
        parsed_data = []
        with open(save_path, 'r') as f:
            lines = f.readlines()
            # 跳过第一行（PoinType=9@6557），从第二行开始解析
            for line in lines[1:]:
                line = line.strip()
                if not line:
                    continue
                
                # 使用正则表达式提取 X, Y, Z, A, B, C, U 的值
                pattern = r'X\s+([-+]?\d+\.?\d*)\$Y\s+([-+]?\d+\.?\d*)\$Z\s+([-+]?\d+\.?\d*)\$A\s+([-+]?\d+\.?\d*)\$B\s+([-+]?\d+\.?\d*)\$C\s+([-+]?\d+\.?\d*)\$U\s+([-+]?\d+\.?\d*)'
                match = re.search(pattern, line)
                if match:
                    values = [float(match.group(i)) for i in range(1, 8)]
                    parsed_data.append(values)
        
        print(f"成功解析 {len(parsed_data)} 行数据")
        
        # 对parsed_data进行采样：保留第一个、最后一个，以及每100个取一个
        original_count = len(parsed_data)
        if len(parsed_data) > 0:
            sampled_indices = set()
            # 保留第一个
            sampled_indices.add(0)
            # 保留最后一个
            sampled_indices.add(len(parsed_data) - 1)
            # 每100个取一个
            for i in range(100, len(parsed_data) - 1, 100):
                sampled_indices.add(i)
            
            # 按索引顺序排序并提取采样后的数据
            sampled_indices = sorted(sampled_indices)
            parsed_data = [parsed_data[i] for i in sampled_indices]
            self.put_in_path = parsed_data # 这样还没有单位转换
            # 将每个值从角度转换为弧度
            parsed_data = [[math.radians(val) for val in row] for row in parsed_data]
            print(f"采样后保留 {len(parsed_data)} 个数据点（原始数据: {original_count} 行），已转换为弧度")

        with self.ui.left_marvin_put_view_simulation:
            slider_name = f"用户{self.client.client_id}_轨迹"
            self.pre_put_moveL_simulation_slider = self.ui.server.gui.add_slider(
                slider_name,
                min=0.0,
                max=len(parsed_data) - 1,  # 最大值为轨迹长度减1（因为索引从0开始）
                step=1,
                initial_value=0,
            )
            
            # 为进度条设置事件处理（槽函数）
            @self.pre_put_moveL_simulation_slider.on_update
            def on_slider_update(event: viser.GuiEvent[viser.GuiSliderHandle]):
                """当进度条值变化时，更新仿真状态"""
                step_index = int(event.target.value)  # 进度条的值直接对应轨迹索引
                num_steps = len(parsed_data)
                step_index = max(0, min(step_index, num_steps - 1))
                joint_config = parsed_data[step_index]
                self.update_robot_visualization(
                    "left_tianji", 
                    joint_config,
                    update_sliders=True,
                    update_end_effector_state=True
                )
        print(f"已为用户 {self.client.client_id} 创建仿真进度条，轨迹长度: {len(parsed_data)}")
        
        # 函数结束后删除临时文件
        if save_path.exists():
            save_path.unlink()
            print(f"已删除临时文件: {save_path}")

    # 左臂放置的进近实机的执行
    def left_marvin_put_in(self, marvin_driver):
        if self.put_in_path is None:
            print("放置的movel路径为空")
            return

        for joint_config in self.put_in_path:
            marvin_driver.clear_set()
            joint_cmd_1=joint_config
            marvin_driver.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
            marvin_driver.send_cmd()
            time.sleep(0.02)  #预留运动时间

    # 左臂放置的退出的实机的执行
    def left_marvin_put_out(self, marvin_driver):
        if self.put_in_path is None:
            print("放置的movel路径为空")
            return

        for joint_config in self.put_in_path[::-1]:
            marvin_driver.clear_set()
            joint_cmd_1=joint_config
            marvin_driver.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
            marvin_driver.send_cmd()
            time.sleep(0.02)  #预留运动时间  