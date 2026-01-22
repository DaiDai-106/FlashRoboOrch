from time import sleep

from robots_orchestra.driver.marvin.robot import Marvin_Robot

robot = Marvin_Robot()
robot.connect("172.32.1.68")

def calculate_modbus_crc(data: bytes) -> bytes:
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for i in range(8):
            if (crc & 0x0001) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def verify_modbus_crc(data: bytes) -> bool:
    recv_crc = data[-2:]
    calc_crc = calculate_modbus_crc(data[:-2])
    return recv_crc == calc_crc


def ch_data_a(data: bytes) -> str:
    init = robot.clear_485_cache('A')
    if init:
        sleep(0.1)
        ret_ch = 2
        send_res = robot.set_485_data('A', data, 8, ret_ch)
        if not send_res[1]:
            print("failed:清除通道A数据失败!")
        sleep(0.1)
        res = 0
        for i in range(6):
            res, data = robot.get_485_data('A', ret_ch)
            if res > 0:
                print("success:接收通道A数据成功!")
                return data
            sleep(0.1)
        return ""


    else:
        print("failed:清除通道A数据失败!")
        return ""


class DH_AG:

    def init_1(self) -> bool:
        """夹爪初始化
        根据初始化方向执行单方向初始化，来寻找最大位置或最小位置
        :return:
            bool:True：成功 False：失败
        """
        data = bytes([0x01, 0x06, 0x01, 0x00, 0x00, 0x01])
        data = data + calculate_modbus_crc(data)
        res = bytes.fromhex(ch_data_a(data))[0:9]
        if res == data:
            print("success:夹爪初始化成功!")
            return True
        else:
            print("failed:夹爪初始化失败!")
            return False

    def init_2(self) -> bool:
        """夹爪初始化
        进行一次张开闭合初始化
        :return:
            bool:True：成功 False：失败
        """
        data = bytes([0x01, 0x06, 0x01, 0x00, 0x00, 0xa5])
        data = data + calculate_modbus_crc(data)
        res = bytes.fromhex(ch_data_a(data))[0:8]
        if res == data:
            print("success:夹爪初始化成功!")
            return True
        else:
            print("failed:夹爪初始化失败!")
            return False

    def set_force(self, value: int) -> bool:
        """设置夹爪力值

        :param value: 力值 20%-100%
        :return:
            bool:True：成功 False：失败
        """
        if 20 > value > 100:
            print("failed:夹爪力值设置错误，范围20-100!")
            return False
        data = bytes([0x01, 0x06, 0x01, 0x01, (value >> 8) & 0xFF, value & 0xFF])
        data = data + calculate_modbus_crc(data)
        res = bytes.fromhex(ch_data_a(data))[0:8]
        if res == data:
            print("success:夹爪力值设置成功!")
            return True
        else:
            print("failed:夹爪力值设置失败!")
            return False

    def set_target_position(self, value: int) -> bool:
        """设置目标位置

        :param value:目标位置值 0-1000
        :return:
            bool:True：成功 False：失败
        """
        if 0 > value > 1000:
            print("failed:夹爪位置设置错误，范围0-1000!")
            return False
        data = bytes([0x01, 0x06, 0x01, 0x03, (value >> 8) & 0xFF, value & 0xFF])
        data = data + calculate_modbus_crc(data)
        res = bytes.fromhex(ch_data_a(data))[0:8]
        if res == data:
            print("success:夹爪目标位置设置成功!")
            return True
        else:
            print("failed:夹爪目标位置设置失败!")
            return False

    def get_force(self) -> int:
        """获取力值

        :return:
            int：力值
        """
        data = bytes([0x01, 0x03, 0x01, 0x01, 0x00, 0x01])
        data = data + calculate_modbus_crc(data)
        res = bytes.fromhex(ch_data_a(data))[0:7]
        is_valid = verify_modbus_crc(res)
        if not is_valid:
            print("failed:夹爪力值获取失败")
            return -1
        return (res[3] << 8) + res[4]

    def get_target_position(self) -> int:
        """获取目标位置

        :return:
            int：目标位置
        """
        data = bytes([0x01, 0x03, 0x01, 0x03, 0x00, 0x01])
        data = data + calculate_modbus_crc(data)
        res = bytes.fromhex(ch_data_a(data))[0:7]
        is_valid = verify_modbus_crc(data)
        if not is_valid:
            print("failed:夹爪目标位置获取失败")
            return -1
        return (res[3] << 8) + res[4]

    def get_init_state(self) -> int:
        """获取初始化状态

        :return:
            int：0：未初始化 1：初始化完成 2：初始化中
        """
        data = bytes([0x01, 0x03, 0x02, 0x00, 0x00, 0x01])
        data = data + calculate_modbus_crc(data)
        res = bytes.fromhex(ch_data_a(data))[0:7]
        is_valid = verify_modbus_crc(data)
        if not is_valid:
            print("failed:夹爪初始化获取失败")
            return -1
        return (res[3] << 8) + res[4]

    def get_hold_state(self) -> int:
        """获取夹持状态

        :return:
            int：0：运动中 1：到达位置 2：夹住物体 3：物体掉落
        """
        data = bytes([0x01, 0x03, 0x02, 0x01, 0x00, 0x01])
        data = data + calculate_modbus_crc(data)
        res = bytes.fromhex(ch_data_a(data))[0:7]
        is_valid = verify_modbus_crc(data)
        if not is_valid:
            print("failed:夹爪夹持状态获取失败")
            return -1
        return (res[3] << 8) + res[4]

    def get_current_position(self) -> int:
        """获取当前位置

        :return:
            int：当前位置
        """
        data = bytes([0x01, 0x03, 0x02, 0x02, 0x00, 0x01])
        data = data + calculate_modbus_crc(data)
        res = bytes.fromhex(ch_data_a(data))[0:7]
        is_valid = verify_modbus_crc(data)
        if not is_valid:
            print("failed:夹爪当前位置获取失败")
            return -1
        return (res[3] << 8) + res[4]

    def get_current(self) -> int:
        """获取电流

        :return:
            int：电流值
        """
        data = bytes([0x01, 0x03, 0x02, 0x04, 0x00, 0x01])
        data = data + calculate_modbus_crc(data)
        res = bytes.fromhex(ch_data_a(data))[0:7]
        is_valid = verify_modbus_crc(data)
        if not is_valid:
            print("failed:夹爪当前电流获取失败")
            return -1
        return (res[3] << 8) + res[4]

    def save_para(self) -> bool:
        """写入保存
        若对夹爪进行过 IO 配置以及 RS485 的参数配置。必须要在此命令下对参数进行FLASH写入保存。
        写入操作会持续 1-2 秒，期间不会响应其他命令，因此建议不要在实时控制中使用此命令
        :return:
            bool:True：成功 False：失败
        """
        data = bytes([0x01, 0x06, 0x03, 0x00, 0x00, 0x01])
        data = data + calculate_modbus_crc(data)
        res = bytes.fromhex(ch_data_a(data))[0:8]
        if res == data:
            print("success:参数保存成功!")
            return True
        else:
            print("failed:参数保存失败!")
            return False

    def set_init_mode(self, value: str) -> bool:
        """设置初始化方向
        初始化结束后停留方向
        已添加写入保存，不要重复操作
        :param value: "open"：张开，"close"：闭合
        :return:
            bool:True：成功 False：失败
        """
        if value == "open":
            value = 0x00
        elif value == "close":
            value = 0x01
        data = bytes([0x01, 0x06, 0x03, 0x01, 0x00, value])
        data = data + calculate_modbus_crc(data)
        res = bytes.fromhex(ch_data_a(data))[0:8]
        if res == data:
            return self.save_para()
        else:
            print("failed:夹爪初始化方向设置失败!")
            return False



def main():
    dh_ag = DH_AG()
    # dh_ag.init_1()
    # sleep(0.5)
    # dh_ag.save_para()
    # sleep(3)
    # return

    dh_ag.set_force(100)
    sleep( 0.5 )
    dh_ag.set_target_position(0)
    sleep( 2 )

    # dh_ag.set_target_position(1000)

if __name__ == "__main__":
    main()
    sleep(1)
    robot.release_robot()