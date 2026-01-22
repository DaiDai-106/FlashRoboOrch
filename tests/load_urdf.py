"""
简单的 viser 程序，用于加载和显示 URDF 机器人模型
同时加载 tianji_car、left_tianji 和 right_tianji
"""
import viser
import numpy as np
import json
import ampl
from viser.extras import ViserUrdf
from viser.extras._urdf import _viser_name_from_frame
from yourdfpy import URDF
from pathlib import Path
from robots_orchestra.planner.ik_solver import IKSolver
from robots_orchestra import SCENE_DIR

# 配置文件路径
CONFIG_PATH = SCENE_DIR / "config.json"

def load_urdf_with_position(server, urdf_path, entity_name, entity_type, position, rotation, 
                             parent_node_name=None, attached_car_urdf=None):
    """加载 URDF 并设置位置
    
    Args:
        server: viser 服务器
        urdf_path: URDF 文件路径
        entity_name: 实体名称
        entity_type: 实体类型 ("robot" 或 "mobile_car")
        position: 位置 [x, y, z]
        rotation: 旋转四元数 [w, x, y, z]
        parent_node_name: 父节点名称（如果附加到其他实体）
        attached_car_urdf: 附加的移动小车 URDF 对象（如果机器人附加到小车上）
    
    Returns:
        (viser_urdf_handle, frame_handle): URDF 可视化句柄和 frame 句柄
    """
    # 加载 URDF
    print(f"正在加载 {entity_type}: {entity_name}")
    urdf = URDF.load(str(urdf_path))
    
    # 确定 frame 名称和 root 节点名称
    if parent_node_name is not None and attached_car_urdf is not None:
        # 机器人附加到移动小车上
        end_effector_link = attached_car_urdf.robot.links[-1]
        if attached_car_urdf.scene is not None:
            prefixed_root = f"{parent_node_name}/visual"
            end_effector_node_name = _viser_name_from_frame(
                attached_car_urdf.scene,
                end_effector_link.name,
                prefixed_root
            )
            frame_name = f"{end_effector_node_name}/robot_frame_{entity_name}"
        else:
            frame_name = f"{parent_node_name}/visual/{end_effector_link.name}/robot_frame_{entity_name}"
    else:
        # 独立实体（移动小车或独立机器人）
        frame_name = f"/{entity_type}_frame_{entity_name}"
    
    # 创建 frame
    frame_handle = server.scene.add_frame(
        name=frame_name,
        show_axes=False,
        position=tuple(position),
        wxyz=tuple(rotation),  # 四元数 (w, x, y, z)
    )
    
    # 创建 root 节点名称
    root_node_name = f"{frame_name}/{entity_name}"
    
    # 在场景中显示 URDF
    viser_urdf = ViserUrdf(
        server,
        urdf,
        root_node_name=root_node_name,
        load_collision_meshes=False
    )
    
    # 设置默认关节配置
    actuated_joints = urdf.actuated_joints
    dof = len(actuated_joints)
    if dof > 0:
        default_joint_config = np.zeros(dof, dtype=np.float64)
        viser_urdf.update_cfg(default_joint_config)
    
    print(f"  ✓ {entity_name} 已加载，自由度: {dof}")
    
    return viser_urdf, frame_handle

def main():
    # 创建 viser 服务器
    server = viser.ViserServer(port=8080, label="Tianji 机器人系统")
    
    # 初始化 IK 求解器
    left_tianji_solver = IKSolver("left_tianji")
    right_tianji_solver = IKSolver("right_tianji")
    
    # 添加网格和坐标系
    server.scene.add_grid(
        name="/world",
        width=15.0,
        height=15.0,
        plane="xy",
        cell_size=0.5,
        section_size=1.0,
    )
    
    server.scene.add_frame(
        name="/world/origin",
        show_axes=True,
        axes_length=1.0,
        axes_radius=0.02,
        position=(0.0, 0.0, 0.0)
    )
    
    # 加载配置文件
    print("正在加载配置文件...")
    with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    mobile_car_config = config.get("mobile_car", {})
    robot_config = config.get("robot_positions", {})
    
    # 存储加载的 URDF 句柄
    urdf_handles = {}
    frame_handles = {}
    end_effector_frames = {}  # 存储末端执行器坐标系
    
    # 1. 先加载移动小车 tianji_car
    tianji_car_config = mobile_car_config.get("tianji_car", {})
    if tianji_car_config:
        tianji_car_path = SCENE_DIR / "mobile" / "tianji_car.urdf"
        tianji_car_urdf_handle, tianji_car_frame = load_urdf_with_position(
            server,
            tianji_car_path,
            "tianji_car",
            "mobile_car",
            tianji_car_config["position"],
            tianji_car_config["rotation"]
        )
        urdf_handles["tianji_car"] = tianji_car_urdf_handle
        frame_handles["tianji_car"] = tianji_car_frame
        
        # 获取移动小车的 URDF 对象（用于后续附加机器人）
        tianji_car_urdf_obj = URDF.load(str(tianji_car_path))
        tianji_car_urdf_obj.update_cfg(np.zeros(len(tianji_car_urdf_obj.actuated_joints), dtype=np.float64))
    else:
        print("警告: 未找到 tianji_car 配置")
        tianji_car_urdf_obj = None
    
    # 2. 加载 left_tianji（附加到 tianji_car）
    left_tianji_config = robot_config.get("left_tianji", {})
    if left_tianji_config and tianji_car_urdf_obj is not None:
        left_tianji_path = SCENE_DIR / "urdf" / "left_tianji.urdf"
        left_tianji_urdf_handle, left_tianji_frame = load_urdf_with_position(
            server,
            left_tianji_path,
            "left_tianji",
            "robot",
            left_tianji_config["position"],
            left_tianji_config["rotation"],
            parent_node_name=frame_handles["tianji_car"].name,
            attached_car_urdf=tianji_car_urdf_obj
        )
        urdf_handles["left_tianji"] = left_tianji_urdf_handle
        frame_handles["left_tianji"] = left_tianji_frame
    
    # 3. 加载 right_tianji（附加到 tianji_car）
    right_tianji_config = robot_config.get("right_tianji", {})
    if right_tianji_config and tianji_car_urdf_obj is not None:
        right_tianji_path = SCENE_DIR / "urdf" / "right_tianji.urdf"
        right_tianji_urdf_handle, right_tianji_frame = load_urdf_with_position(
            server,
            right_tianji_path,
            "right_tianji",
            "robot",
            right_tianji_config["position"],
            right_tianji_config["rotation"],
            parent_node_name=frame_handles["tianji_car"].name,
            attached_car_urdf=tianji_car_urdf_obj
        )
        urdf_handles["right_tianji"] = right_tianji_urdf_handle
        frame_handles["right_tianji"] = right_tianji_frame
    
    print("\n所有模型加载完成！")
    print(f"访问 http://localhost:8080 查看机器人系统")
    
    # 创建关节控制 slider
    with server.gui.add_folder("关节控制"):
        for entity_name, urdf_handle in urdf_handles.items():
            urdf_obj = urdf_handle._urdf
            actuated_joints = urdf_obj.actuated_joints
            dof = len(actuated_joints)
            
            if dof > 0:
                with server.gui.add_folder(entity_name, expand_by_default=False):
                    sliders = {}
                    joint_names = [joint.name for joint in actuated_joints]
                    
                    # 先创建所有 slider
                    for joint_name in joint_names:
                        slider = server.gui.add_slider(
                            label=joint_name,
                            min=-3.14,
                            max=3.14,
                            step=0.01,
                            initial_value=0.0,
                        )
                        sliders[joint_name] = slider
                    
                    # 创建更新函数（在 slider 都创建之后）
                    def update_robot_config():
                        """更新机器人关节配置和末端执行器坐标系"""
                        current_config = np.array([
                            sliders[joint_names[j]].value for j in range(len(joint_names))
                        ], dtype=np.float64)
                        urdf_handle.update_cfg(current_config)
                        
                        # 更新末端执行器坐标系（仅对机器人）
                        if entity_name in end_effector_frames:
                            update_end_effector_frame(entity_name, urdf_obj, current_config, 
                                                     urdf_handle, frame_handles.get(entity_name), end_effector_frames)
                    
                    # 为每个 slider 绑定更新回调
                    for joint_name in joint_names:
                        slider = sliders[joint_name]
                        
                        @slider.on_update
                        def on_slider_update(event: viser.GuiEvent[viser.GuiSliderHandle]):
                            """当 slider 值变化时，更新对应的关节配置"""
                            update_robot_config()
    
    # 定义更新末端执行器坐标系的函数
    def update_end_effector_frame(robot_name, urdf_obj, joint_config, urdf_handle, robot_frame, end_effector_frames_dict):
        """更新末端执行器坐标系的位置和旋转"""
        if robot_name not in end_effector_frames_dict:
            return
        
        # 获取末端执行器链接（最后一个链接）
        end_effector_link = urdf_obj.robot.links[-1]
        
        # 更新 URDF 配置
        urdf_obj.update_cfg(joint_config)
        
        # 获取末端执行器相对于 base 的变换
        tf_end_effector_local = urdf_obj.get_transform(end_effector_link.name)
        tf_end_effector_local = np.array(tf_end_effector_local, dtype=np.float64, order='C', copy=True)
        
        # 获取机器人 frame 的世界变换
        if robot_frame is not None:
            robot_position = np.array(robot_frame.position, dtype=np.float64)
            robot_rotation = np.array(robot_frame.wxyz, dtype=np.float64)  # (w, x, y, z)
            
            # 构建机器人 frame 的世界变换矩阵
            qt7_robot_world = np.array([
                robot_rotation[1],  # qx
                robot_rotation[2],  # qy
                robot_rotation[3],  # qz
                robot_rotation[0],  # qw
                robot_position[0],  # x
                robot_position[1],  # y
                robot_position[2]   # z
            ], dtype=np.float64)
            tf_robot_world = ampl.qt7_to_tf44(qt7_robot_world)
            
            # 计算末端执行器的世界变换
            tf_end_effector_world = tf_robot_world @ tf_end_effector_local
            
            # 提取世界坐标系下的位置和旋转
            qt7_end_effector_world = ampl.tf44_to_qt7(tf_end_effector_world)
            
            # 更新末端执行器坐标系
            end_effector_frame = end_effector_frames_dict[robot_name]
            end_effector_frame.position = (
                float(qt7_end_effector_world[4]),
                float(qt7_end_effector_world[5]),
                float(qt7_end_effector_world[6])
            )
            end_effector_frame.wxyz = (
                float(qt7_end_effector_world[3]),  # w
                float(qt7_end_effector_world[0]),  # x
                float(qt7_end_effector_world[1]),  # y
                float(qt7_end_effector_world[2])   # z
            )
    
    # 为 left_tianji 和 right_tianji 创建末端执行器坐标系
    for robot_name in ["left_tianji", "right_tianji"]:
        if robot_name in urdf_handles:
            urdf_handle = urdf_handles[robot_name]
            urdf_obj = urdf_handle._urdf
            robot_frame = frame_handles.get(robot_name)
            
            # 获取末端执行器链接
            end_effector_link = urdf_obj.robot.links[-1]
            
            # 初始化关节配置（全零）
            default_config = np.zeros(len(urdf_obj.actuated_joints), dtype=np.float64)
            urdf_obj.update_cfg(default_config)
            
            # 获取末端执行器相对于 base 的变换
            tf_end_effector_local = urdf_obj.get_transform(end_effector_link.name)
            tf_end_effector_local = np.array(tf_end_effector_local, dtype=np.float64, order='C', copy=True)
            
            # 计算世界坐标
            if robot_frame is not None:
                robot_position = np.array(robot_frame.position, dtype=np.float64)
                robot_rotation = np.array(robot_frame.wxyz, dtype=np.float64)
                
                qt7_robot_world = np.array([
                    robot_rotation[1], robot_rotation[2], robot_rotation[3], robot_rotation[0],
                    robot_position[0], robot_position[1], robot_position[2]
                ], dtype=np.float64)
                tf_robot_world = ampl.qt7_to_tf44(qt7_robot_world)
                tf_end_effector_world = tf_robot_world @ tf_end_effector_local
                qt7_end_effector_world = ampl.tf44_to_qt7(tf_end_effector_world)
                
                # 创建末端执行器坐标系
                end_effector_frame = server.scene.add_frame(
                    name=f"/end_effector_{robot_name}",
                    show_axes=True,
                    axes_length=0.15,
                    axes_radius=0.01,
                    position=(
                        float(qt7_end_effector_world[4]),
                        float(qt7_end_effector_world[5]),
                        float(qt7_end_effector_world[6])
                    ),
                    wxyz=(
                        float(qt7_end_effector_world[3]),  # w
                        float(qt7_end_effector_world[0]),  # x
                        float(qt7_end_effector_world[1]),  # y
                        float(qt7_end_effector_world[2])   # z
                    )
                )
                end_effector_frames[robot_name] = end_effector_frame
                print(f"  ✓ {robot_name} 末端执行器坐标系已创建")
    
    # 保持服务器运行
    while True:
        import time
        time.sleep(1)

if __name__ == "__main__":
    main()
