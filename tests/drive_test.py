# 驱动测试 
import json
import time
from ctypes import *
from pathlib import Path
from robots_orchestra.driver.marvin.robot import Marvin_Robot , DCSS

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

def main():
    dcss=DCSS()
    marvin = Marvin_Robot()
    
    code = marvin.connect("172.32.1.68")
    marvin.clear_set()
    marvin.clear_error("A")
    marvin.clear_error("B")
    marvin.send_cmd()
    time.sleep(1)

    motion_tag = 0
    frame_update = None
    for i in range(5):
        sub_data = marvin.subscribe(dcss)
        print(f"connect frames :{sub_data['outputs'][0]['frame_serial']}")
        if sub_data['outputs'][0]['frame_serial'] != 0 and frame_update != sub_data['outputs'][0]['frame_serial']:
            motion_tag += 1
            frame_update = sub_data['outputs'][0]['frame_serial']
        time.sleep(0.1)
    if motion_tag > 0:
        print('success:机器人连接成功!')
    else:
        print('failed:机器人连接失败!')
        exit(0)

    sub_data = marvin.subscribe(dcss)
    joint_data = getJointData( sub_data, 0 )
    print(joint_data)

    code = marvin.release_robot()
    exit( 0 )


    marvin.clear_set()
    marvin.set_state(arm='A',state=1)          # state=3扭矩模式
    marvin.set_vel_acc(arm='A',velRatio=2, AccRatio=1)
    marvin.send_cmd()
    time.sleep(0.5)


    marvin.clear_set()
    joint_cmd_1=[0.288520,0.309527,0.383520,0.324928,0.255664,-0.052320]
    marvin.set_joint_cmd_pose(arm='A',joints=joint_cmd_1)
    marvin.send_cmd()

    print("请按回车继续...")
    input()
    code = marvin.release_robot()
    exit( 0 )



    # 尝试力控
    marvin.clear_set()
    marvin.set_state(arm='A',state=3)          # state=3扭矩模式
    marvin.set_impedance_type(arm='A',type=3)  # type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    marvin.send_cmd()

    # 设置力控参数
    marvin.clear_set()
    marvin.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 1, 0, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                        fcAdjLmt=10.0)
    time.sleep(0.5)


    marvin.clear_set()
    code = marvin.set_force_cmd(arm='A',f=2)

    # '''订阅数据查看力控指令是否设置成功'''
    # sub_data=marvin.subscribe(dcss)

    # 这里下发一个轴关节的指令来运动
    marvin.clear_set()
    joint_cmd_2=[49.2092, -48.5245, -42.5652, -51.8185, 67.1795, -23.4639, -31.5124]
    marvin.set_joint_cmd_pose(arm='A',joints=joint_cmd_2)
    marvin.send_cmd()
    time.sleep(10) #预留运动时间


     # 设置力控参数
    marvin.clear_set()
    marvin.set_force_control_params(arm='A',fcType=0, fxDirection=[0, 0, 1, 0, 0, 0], fcCtrlpara=[0, 0, 0, 0, 0, 0, 0],
                                        fcAdjLmt=50.0)
    time.sleep(0.5)

    marvin.clear_set()
    joint_cmd_2=[48.702, -42.903, -37.5625, -49.4006, 70.4727, -30.6254, -32.8553]
    marvin.set_joint_cmd_pose(arm='A',joints=joint_cmd_2)
    marvin.send_cmd()
    time.sleep(10) #预留运动时间

    print("运动结束")
    input()

    code = marvin.release_robot()
    print(code)

if __name__ == "__main__":
    main()
