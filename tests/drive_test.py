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


def fk( joint ):

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


def calculate( t1, t2):
    x1 = t1[ 0 ]
    y1 = t1[1]
    z1 = t1[2]
    
    x2 = t2[0]
    y2 = t2[1]
    z2 = t2[2]
    
    # 计算差向量
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    
    # 计算欧几里得范数（norm）
    norm = math.sqrt(dx*dx + dy*dy + dz*dz)
    
    return norm


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
print(joint_data)

# 设置力控模式
robot.clear_set()
robot.set_state(arm='A',state=3)#state=3扭矩模式
robot.set_impedance_type(arm='A',type=3) #type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
robot.send_cmd()
time.sleep(0.5)


# 第一步，目前的尝试就是需要先堆到一个面上
face_touch = False
transform = fk( joint_data)
while not face_touch:
    '''设置力控参数'''
    robot.clear_set()
    robot.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 1, 0, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                            fcAdjLmt=5.0)
    robot.send_cmd()
    time.sleep(0.5)

    '''设置力控指令'''
    robot.clear_set()
    #根据前面设置的力控参数，这里力控的效果是：
    #在Y轴方向有个10N的力一直压着手臂,相对于基座,末端往下压了5厘米的效果， 上下拖动手臂试试， 手臂像弹簧一样会回到原来的位置。力控阻抗下更柔顺
    robot.set_force_cmd(arm='A',f= 5)
    robot.send_cmd()
    time.sleep(1)

    sub_data=robot.subscribe(dcss)
    # logger.info(f'set force = {sub_data["outputs"][0]["est_joint_force"]}')
    current_joints = getJointData( sub_data, 0 )
    time.sleep(0.2)

    curent_transform = fk( current_joints)
    distance = calculate(transform, curent_transform)
    print(f'distance: {distance}')
    transform = curent_transform

    if distance < 0.1:
        face_touch = True
        print("face touch")
        break

    if sub_data['states'][0]['err_code'] == 6:
        face_touch = False
        print("wu zu kang")
        break


if not face_touch:
    print("face not touch")
    exit(0)

# 第二步，通过Z形的运动方式进行简单的寻力
z_touch = False  # 插拔的进近状态
y_touch = False  # 重力方向的状态
# step = 50;

while not z_touch or not y_touch:
    z_touch = False
    y_touch = False
    if not z_touch:
        robot.clear_set()
        robot.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 0, 1, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                                fcAdjLmt=10)
        robot.send_cmd()
        time.sleep(0.5)

        '''设置力控指令'''
        robot.clear_set()
        #根据前面设置的力控参数，这里力控的效果是：
        #在Y轴方向有个10N的力一直压着手臂,相对于基座,末端往下压了5厘米的效果， 上下拖动手臂试试， 手臂像弹簧一样会回到原来的位置。力控阻抗下更柔顺
        robot.set_force_cmd(arm='A',f=20)
        robot.send_cmd()
        time.sleep(2)

        sub_data=robot.subscribe(dcss)
        current_joints = getJointData( sub_data, 0 )
        time.sleep(0.2)
        curent_transform = fk( current_joints)
        distance = calculate(transform, curent_transform)
        print(f'distance: {distance}')
        transform = curent_transform
        if distance < 0.1:
            z_touch = True
            print("z touch")

    if not y_touch:
        robot.clear_set()
        robot.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 1, 0, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                                fcAdjLmt=5.0)
        robot.send_cmd()
        time.sleep(0.5)

        robot.clear_set()
        #根据前面设置的力控参数，这里力控的效果是：
        #在Y轴方向有个10N的力一直压着手臂,相对于基座,末端往下压了5厘米的效果， 上下拖动手臂试试， 手臂像弹簧一样会回到原来的位置。力控阻抗下更柔顺
        robot.set_force_cmd(arm='A',f=20)
        robot.send_cmd()
        time.sleep(1)

        sub_data=robot.subscribe(dcss)
        current_joints = getJointData( sub_data, 0 )
        time.sleep(0.2)
        curent_transform = fk( current_joints)
        distance = calculate(transform, curent_transform)
        print(f'distance: {distance}')
        transform = curent_transform
        if distance < 0.1:
            y_touch = True
            print("y touch")

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