# 驱动测试 
import json
import time
import math
from ctypes import *
from pathlib import Path
import logging
from robots_orchestra.driver.marvin.robot import Marvin_Robot , DCSS
from robots_orchestra.driver.marvin.kinematics import Marvin_Kine

logging.basicConfig(format='%(message)s')
logger = logging.getLogger('debug_printer')
logger.setLevel(logging.INFO)# 一键关闭所有调试打印
logger.setLevel(logging.DEBUG)  # 默认开启DEBUG级


def compute_fk_pose(joint):
    """计算正向运动学，返回6D pose [x, y, z, a, b, c]

    Args:
        joint: 关节角度列表
    
    Returns:
        6D pose列表 [x, y, z, a, b, c]
    """
    # 使用实例运动学对象
    kine_obj = kk;

    # 将关节角度转换为浮点数列表
    list_joints = [float(j) for j in joint]

    # 正向运动学：关节角度 -> 4x4矩阵
    fk_mat = kine_obj.fk(robot_serial=0, joints=list_joints)

    if not fk_mat:
        return [0.0] * 6

    # 4x4矩阵 -> XYZABC格式
    pose_6d = kine_obj.mat4x4_to_xyzabc(pose_mat=fk_mat)

    if not pose_6d:
        return [0.0] * 6

    return pose_6d

# 为了保持向后兼容，创建一个别名
fk = compute_fk_pose


def extract_position(transform):
    """从变换矩阵或6D pose中提取位置 [x, y, z]
    
    Args:
        transform: 可以是4x4矩阵（列表的列表）或6D pose [x, y, z, a, b, c]
    
    Returns:
        [x, y, z] 位置列表
    """
    # 检查是否是4x4矩阵格式（嵌套列表，且第一行有4个元素）
    if isinstance(transform, list) and len(transform) > 0:
        if isinstance(transform[0], list) and len(transform[0]) >= 4:
            # 4x4矩阵格式：位置在 [0][3], [1][3], [2][3]
            return [transform[0][3], transform[1][3], transform[2][3]]
        elif len(transform) >= 3:
            # 6D pose格式：[x, y, z, a, b, c] 或 [x, y, z]
            return [transform[0], transform[1], transform[2]]
    return [0.0, 0.0, 0.0]


def calculate( t1, t2, mode = 'norm' ):
    """计算两个变换之间的欧几里得距离
    
    Args:
        t1, t2: 可以是4x4矩阵或6D pose格式
    
    Returns:
        欧几里得距离（毫米）
    """
    pos1 = extract_position(t1)
    pos2 = extract_position(t2)
    
    x1, y1, z1 = pos1
    x2, y2, z2 = pos2
    
    # 计算差向量
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    
    if mode == 'norm':
        # 计算欧几里得范数（norm）
        norm = math.sqrt(dx*dx + dy*dy + dz*dz)
        return norm
    elif mode == 'x':
        return abs(x2 - x1)
    elif mode == 'y':
        return abs(y2 - y1)
    elif mode == 'z':
        return abs(z2 - z1)
    
    return norm


def calculate_directional_distance(t1, t2, direction='x'):
    """计算特定方向的距离
    
    Args:
        t1, t2: 可以是4x4矩阵或6D pose格式
        direction: 'x', 'y', 'z' 分别对应X、Y、Z方向
    
    Returns:
        该方向的距离（毫米）
    """
    pos1 = extract_position(t1)
    pos2 = extract_position(t2)
    
    if direction == 'x':
        return abs(pos2[0] - pos1[0])
    elif direction == 'y':
        return abs(pos2[1] - pos1[1])
    elif direction == 'z':
        return abs(pos2[2] - pos1[2])
    else:
        return 0.0


def calculate_adj_lmt(distance, min_lmt=5.0, max_lmt=30.0):
    """根据距离动态计算调整范围(fcAdjLmt)
    
    Args:
        distance: 当前距离（毫米）
        min_lmt: 最小调整范围（毫米），默认5.0
        max_lmt: 最大调整范围（毫米），默认30.0
    
    Returns:
        合适的调整范围值
    """
    # 距离远时使用较大范围，距离近时使用较小范围
    # 使用分段线性映射
    if distance > 50.0:
        # 距离 > 50mm，使用较大范围
        adj_lmt = min(max_lmt, 20.0 + (distance - 50.0) * 0.2)
    elif distance > 20.0:
        # 距离 20-50mm，使用中等范围
        adj_lmt = 10.0 + (distance - 20.0) * 0.33
    elif distance > 10.0:
        # 距离 10-20mm，使用中等范围
        adj_lmt = 5.0 + (distance - 10.0) * 0.5
    else:
        # 距离 < 10mm，使用较小范围
        adj_lmt = min_lmt + distance * 0.3
    
    # 确保在合理范围内
    adj_lmt = max(min_lmt, min(max_lmt, adj_lmt))
    return adj_lmt


def getJointData(currentData, arm_index, data_type="pos"):
    """获取关节数据"""
    if arm_index < len(currentData["outputs"]):
        output = currentData["outputs"][arm_index]
        data_map = {
            "pos": output.get("fb_joint_pos", [0.0] * 7),
            "vel": output.get("fb_joint_vel", [0.0] * 7),
            "sToq": output.get("fb_joint_sToq", [0.0] * 7),
            "cToq": output.get("fb_joint_cToq", [0.0] * 7),
            "them": output.get("fb_joint_them", [0.0] * 7),
        }
        return data_map.get(data_type, [0.0] * 7)
    return [0.0] * 7


'''初始化订阅数据的结构体'''
dcss=DCSS()

'''初始化机器人接口'''
robot=Marvin_Robot()
kk = Marvin_Kine()
cp = "src/robots_orchestra/driver/config/ccs_m6.MvKDCfg"
if cp:
    ini_result = kk.load_config(config_path=cp)
    # print(f"ini_results:{ini_result}")
    if ini_result:
        kk.initial_kine(
            robot_serial=0,
            robot_type=ini_result["TYPE"][0],
            dh=ini_result["DH"][0],
            pnva=ini_result["PNVA"][0],
            j67=ini_result["BD"][0],
        )

'''查验连接是否成功'''
init = robot.connect('172.32.1.68')
if init==0:
    logger.error('failed:端口占用，连接失败!')
    exit(0)
else:
    '''防总线通信异常,先清错'''
    time.sleep(0.5)
    robot.clear_set()
    robot.clear_error('A')
    robot.clear_error('B')
    robot.send_cmd()
    time.sleep(0.5)

    motion_tag = 0
    frame_update = None
    for i in range(5):
        sub_data = robot.subscribe(dcss)
        print(f"connect frames :{sub_data['outputs'][0]['frame_serial']}")
        if sub_data['outputs'][0]['frame_serial'] != 0 and frame_update != sub_data['outputs'][0]['frame_serial']:
            motion_tag += 1
            frame_update = sub_data['outputs'][0]['frame_serial']
        time.sleep(0.1)
    if motion_tag > 0:
        logger.info('success:机器人连接成功!')
    else:
        logger.error('failed:机器人连接失败!')
        exit(0)


'''开启日志以便检查'''
robot.log_switch('1') #全局日志开关
robot.local_log_switch('1') # 主要日志


'''清错'''
robot.clear_set()
robot.clear_error('A')
robot.send_cmd()
time.sleep(1)


# 读取关节状态
sub_data = robot.subscribe(dcss)
joint_data = getJointData( sub_data, 0 )
# print(f'joint_data: {joint_data}')
# exit( 0 )

joint_data =[160.0338, 67.3775, -95.272, -82.2662, 15.7409, -21.2089, 41.7361]
ref_fk_matrix = kk.fk(robot_serial=0, joints=joint_data)  # 4x4矩阵
print(f'ref_fk_matrix: {ref_fk_matrix}')
ref_fk_matrix[0][3] = 410
ref_fk_matrix[1][3] = 250
ref_fk_matrix[2][3] = 190

ik = kk.ik(robot_serial=0, pose_mat=ref_fk_matrix, ref_joints=joint_data)
print(f'ik: {ik.m_Output_RetJoint.to_list()}')

joint_data = ik.m_Output_RetJoint.to_list()
print(f'joint_data: {joint_data}')
print(f'type(joint_data): {type(joint_data)}')
fk_pose_result = fk(joint=joint_data)  # 调用函数，返回6D pose
print(f'fk_pose_result: {fk_pose_result}')

# 设置力控模式
robot.clear_set()
robot.set_state(arm='A',state=3)#state=3扭矩模式
robot.set_impedance_type(arm='A',type=3) #type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
robot.send_cmd()
time.sleep(0.5)


# 第一步，目前的尝试就是需要先堆到一个面上
face_touch = False
transform = fk( joint_data)
update_pose = ref_fk_matrix.copy()

robot.clear_set()
robot.set_joint_cmd_pose(arm='A',joints=joint_data)
robot.send_cmd()
time.sleep(0.5)


# 这里以逆解的方向不断的缩小Z，一开始是50， 然后慢慢的通过Y方向的移动来卡住它
i = 1
while not face_touch:
    '''设置力控参数'''
    robot.clear_set()
    distance = 80.0

    code = robot.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 0, 1, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                            fcAdjLmt =distance )
    robot.send_cmd()
    time.sleep(0.5)

    '''设置力控指令'''
    robot.clear_set()
    robot.set_force_cmd(arm='A',f=2)
    robot.send_cmd()
    time.sleep(0.5)

    sub_data=robot.subscribe(dcss)
    current_joints = getJointData( sub_data, 0 )
    curent_transform = fk( current_joints)
    distance = calculate(transform, curent_transform, mode='z')
    print(f'distance: {distance}')
    transform = curent_transform
    # break

    update_pose[2][3] = 190 - i * 10
    ik = kk.ik(robot_serial=0, pose_mat=update_pose, ref_joints=joint_data)
    # print(f'ik: {ik.m_Output_RetJoint.to_list()}')
    joint_data = ik.m_Output_RetJoint.to_list()
    robot.clear_set()
    robot.set_joint_cmd_pose(arm='A',joints=joint_data)
    robot.send_cmd()
    time.sleep(0.5)
    i += 1

    if i > 4 and distance < 9:
        face_touch = True
        print("face touch")
        break

    if sub_data['states'][0]['err_code'] == 6:
        face_touch = False
        print("wu zu kang")
        break

# exit( 0 )

if not face_touch:
    print("face not touch")
    exit(0)

# 第二步，通过Z形的运动方式进行简单的寻力
z_touch = False  # 插拔的进近状态
y_touch = False  # 重力方向的状态

# 设置目标位置（transform 是 6D pose 格式 [x, y, z, a, b, c]）
# 确保 transform 是列表格式，如果是 4x4 矩阵则转换为 6D pose
if isinstance(transform, list) and len(transform) > 0:
    if isinstance(transform[0], list):
        # 如果是 4x4 矩阵，提取位置并转换为 6D pose
        pos = extract_position(transform)
        transform = pos + [0.0, 0.0, 0.0]  # [x, y, z, a, b, c]
    elif len(transform) < 6:
        # 如果长度不足6，补齐到6D pose
        transform = list(transform) + [0.0] * (6 - len(transform))
    else:
        transform = list(transform)
else:
    # 如果 transform 不是列表，尝试提取位置
    pos = extract_position(transform)
    transform = pos + [0.0, 0.0, 0.0]

# 保存初始位置作为参考
initial_transform = transform.copy() if hasattr(transform, 'copy') else list(transform)

# 为每个方向保存上一次的位置
prev_x_transform = list(transform)
prev_y_transform = list(transform)

while not z_touch or not y_touch:
    # z_touch和y_touch在循环中保持状态，不会被重置
    if not y_touch:
        # 获取当前位置
        sub_data=robot.subscribe(dcss)
        current_joints = getJointData( sub_data, 0 )
        current_transform = kk.fk( robot_serial=0, joints=current_joints)
        # current_transform[2][3] = update_pose[2][3]


        joint_cmd_1 = getJointData( sub_data, 0 )
        robot.clear_set()
        robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
        robot.send_cmd()
        time.sleep(0.5)
        
        robot.clear_set()
        robot.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 1, 0, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                                fcAdjLmt=1)
        robot.send_cmd()
        time.sleep(0.5)

        '''设置力控指令'''
        robot.clear_set()
        #根据前面设置的力控参数，这里力控的效果是：
        #在X轴方向根据距离动态调整搜索范围，距离远时范围大，距离近时范围小
        robot.set_force_cmd(arm='A',f=1)
        robot.send_cmd()
        time.sleep(0.5)

        sub_data=robot.subscribe(dcss)
        current_joints = getJointData( sub_data, 0 )
        time.sleep(0.2)

        # continue
        
    if not z_touch:
        # continue
        current_transform = kk.fk( robot_serial=0, joints=current_joints)
        current_transform[2][3] = update_pose[2][3]
        ik = kk.ik(robot_serial=0, pose_mat=current_transform, ref_joints=joint_data)
        joint_data = ik.m_Output_RetJoint.to_list()
        
        robot.clear_set()
        robot.set_joint_cmd_pose(arm='A',joints=joint_data)
        robot.send_cmd()
        time.sleep(0.5)
        
        robot.clear_set()
        robot.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 0, 1, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                                fcAdjLmt=75)
        robot.send_cmd()
        time.sleep(0.5)

        robot.clear_set()
        #根据前面设置的力控参数，这里力控的效果是：
        #在Y轴方向根据距离动态调整搜索范围，距离远时范围大，距离近时范围小
        #如果X方向已接触，则使用更大的力和更小的范围
        robot.set_force_cmd(arm='A',f=1)
        robot.send_cmd()
        time.sleep(1)

# 然后终止退出。
exit( 0 )


#---------------------------------------------------------------上面是真正意义上的力控尝试-----------
# exit(0)

# tool_result=robot.get_tool_info()
# print(f'tool_result:{tool_result}')
# print(type(tool_result))

# logger.info('success, 机器人已设置左右臂的工具信息.')
# if isinstance(tool_result[0], list):
#     tool_dyn_l = ""
#     tool_dyn_r = ""
#     tool_kine_l = ""
#     tool_kine_r = ""
#     for i in range(10):
#         tool_dyn_l += f"{tool_result[0][i]:.3f},"
#         tool_dyn_r += f"{tool_result[1][i]:.3f},"
#         if i < 6:
#             tool_kine_l += f"{tool_result[0][10 + i]:.3f},"
#             tool_kine_r += f"{tool_result[1][10 + i]:.3f},"
#     tool_dyn_l = tool_dyn_l.rstrip(", ")
#     tool_dyn_r = tool_dyn_r.rstrip(", ")
#     tool_kine_l = tool_kine_l.rstrip(", ")
#     tool_kine_r = tool_kine_r.rstrip(", ")

# print(f'tool_dyn_l:{tool_result[0][:10]}')

    # 从控制器加载的工具信息
    # robot.clear_set()
    # robot.set_tool(arm='A', dynamicParams=tool_result[0][:10], kineParams=tool_result[0][10:])
    # robot.send_cmd()
    # time.sleep(0.5)
    # robot.set_tool(arm='B', dynamicParams=tool_result[1][:10], kineParams=tool_result[1][10:])


# joint_cmd_1=[0, 0, 0, 0, 0, 0, 0]
# robot.clear_set()
# robot.set_state(arm='A',state=1)          # state=3扭矩模式
# robot.set_vel_acc(arm='A',velRatio=4, AccRatio=1)
# robot.send_cmd()
# time.sleep(0.5)

# robot.clear_set()
# robot.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
# robot.send_cmd()

# time.sleep(10)
# sub_data=robot.subscribe(dcss)
# # logger.info(f'set force cmd={sub_data["inputs"][0]["force_cmd"]}')
# # open('sub_data_1.json', 'w').write(str(sub_data))
# '''下使能'''
# robot.clear_set()
# robot.set_state(arm='A',state=0)
# robot.send_cmd()

# '''释放机器人内存'''
# robot.release_robot()
# exit(0)
# input()


'''设置扭矩模式 力控模式 '''
robot.clear_set()
robot.set_state(arm='A',state=3)#state=3扭矩模式
robot.set_impedance_type(arm='A',type=3) #type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
robot.send_cmd()
time.sleep(0.5)

# '''设置力控参数'''
# robot.clear_set()
# # 设置是在Y轴方向有5厘米的调节范围
# robot.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 1, 0, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
#                                         fcAdjLmt=20.0)
# robot.send_cmd()
# time.sleep(0.5)

'''订阅数据查看是否设置'''
sub_data=robot.subscribe(dcss)
logger.info(f"current state{sub_data['states'][0]['cur_state']}")
logger.info(f'set impedance type={sub_data["inputs"][0]["imp_type"]}')
logger.info(f"arm error code:{sub_data['states'][0]['err_code']}")
logger.info(f'set force fcType={sub_data["inputs"][0]["force_type"]}, '
             f'fxDirection={sub_data["inputs"][0]["force_dir"][:]}, '
             f'fcCtrlpara={sub_data["inputs"][0]["force_pidul"][:]}, '
             f'fcAdjLmt={sub_data["inputs"][0]["force_adj_lmt"]}')



# '''设置力控指令'''
# robot.clear_set()
# #根据前面设置的力控参数，这里力控的效果是：
# #在Y轴方向有个10N的力一直压着手臂,相对于基座,末端往下压了5厘米的效果， 上下拖动手臂试试， 手臂像弹簧一样会回到原来的位置。力控阻抗下更柔顺
# robot.set_force_cmd(arm='A',f=20)
# robot.send_cmd()
# time.sleep(0.5)

# '''订阅数据查看力控指令是否设置成功'''
# sub_data=robot.subscribe(dcss)
# logger.info(f'set force cmd={sub_data["inputs"][0]["force_cmd"]}')
# open('sub_data.json', 'w').write(str(sub_data))


start_time = time.time()
duration = 12  # 运行30秒
while time.time() - start_time < duration:
    '''设置力控参数'''
    robot.clear_set()
    # 设置是在Y轴方向有5厘米的调节范围
    robot.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 1, 0, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                            fcAdjLmt=20.0)
    robot.send_cmd()
    time.sleep(0.5)


    '''设置力控指令'''
    robot.clear_set()
    #根据前面设置的力控参数，这里力控的效果是：
    #在Y轴方向有个10N的力一直压着手臂,相对于基座,末端往下压了5厘米的效果， 上下拖动手臂试试， 手臂像弹簧一样会回到原来的位置。力控阻抗下更柔顺
    robot.set_force_cmd(arm='A',f=10)
    robot.send_cmd()
    time.sleep(2)

    sub_data=robot.subscribe(dcss)
    logger.info(f'set force = {sub_data["outputs"][0]["est_joint_force"]}')
    time.sleep(0.2)
    if sub_data['states'][0]['err_code'] == 6:
        print("wu zu kang")
        break

start_time = time.time()
duration = 12  # 运行30秒
while time.time() - start_time < duration:
    '''设置力控参数'''
    robot.clear_set()
    # 设置是在Y轴方向有5厘米的调节范围
    robot.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 0, 1, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                            fcAdjLmt=20.0)
    robot.send_cmd()
    time.sleep(0.5)


    '''设置力控指令'''
    robot.clear_set()
    #根据前面设置的力控参数，这里力控的效果是：
    #在Y轴方向有个10N的力一直压着手臂,相对于基座,末端往下压了5厘米的效果， 上下拖动手臂试试， 手臂像弹簧一样会回到原来的位置。力控阻抗下更柔顺
    robot.set_force_cmd(arm='A',f=10)
    robot.send_cmd()
    time.sleep(2)

    sub_data=robot.subscribe(dcss)
    logger.info(f'set force = {sub_data["outputs"][0]["est_joint_force"]}')
    time.sleep(0.2)
    if sub_data['states'][0]['err_code'] == 6:
        print("wu zu kang")
        break

# time.sleep(30)#预留时间拖拽时间：上下拖动手臂试试， 手臂像弹簧一样会回到原来的位置。力控阻抗下更柔顺

'''下使能'''
robot.clear_set()
# robot.set_state(arm='A',state=0)
# robot.send_cmd()
# time.sleep(0.5)

'''释放机器人内存'''
robot.release_robot()