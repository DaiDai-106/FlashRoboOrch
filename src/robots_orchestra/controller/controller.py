from typing_extensions import Self
import viser
from typing import Dict
from robots_orchestra.controller.usr_session import UserSession
from robots_orchestra.driver.marvin.robot import Marvin_Robot, DCSS
from robots_orchestra.driver.marvin.kinematics import Marvin_Kine
from robots_orchestra.viz.viser import ViserUI
import time

class Controller:
    """控制器：负责事件分发和协调各组件"""
    def __init__(self, viser_ui: ViserUI):
        self.ui = viser_ui
        self.marvin_driver = Marvin_Robot()
        self.marvin_kine = Marvin_Kine()
        self.marvin_kine_2 = Marvin_Kine()
        self.initialize_marvin()
        
        self.sessions: Dict[viser.ClientHandle, UserSession] = {}

        @self.ui.server.on_client_connect
        def _(client: viser.ClientHandle):
            self.handle_connect(client)
            self.set_default_camera(client)

        @self.ui.server.on_client_disconnect
        def _(client: viser.ClientHandle):
            self.handle_disconnect(client)        

        # 天机的组装任务，icp匹配结果可能并不着急，主要先示教好过渡段的姿态
        @self.ui.right_marvin_home.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_right_marvin_home(event)
        
        @self.ui.right_marvin_first_capture.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_right_marvin_first_capture(event)
        
        @self.ui.right_marvin_first_icp.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_right_marvin_first_icp(event)
        
        @self.ui.right_marvin_second_capture.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_right_marvin_second_capture(event)
        
        @self.ui.right_marvin_second_icp.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_right_marvin_second_icp(event)

        @self.ui.left_marvin_home.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_home(event)

        @self.ui.left_marvin_pre_zhua_put_movel.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_pre_zhua_put_movel(event)

        # 桌面组配左臂抓取任务
        @self.ui.left_marvin_zhua_pose.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_zhua_pose(event)

        # 求解到抓取的movel
        @self.ui.left_marvin_zhua_l.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_zhua_l(event)


        @self.ui.left_marvin_pre_zhua_move.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_pre_zhua_move(event)
        
        @self.ui.left_marvin_pre_zhua_to_put.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_pre_zhua_to_put(event)

        @self.ui.left_marvin_pre_zhua_to_put_reverse.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_pre_zhua_to_put_reverse(event)

        # 左臂进近状态
        @self.ui.left_marvin_zhua_in.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_zhua_in(event)

        # 左臂退出状态
        @self.ui.left_marvin_zhua_out.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_zhua_out(event)

        #-------------------------------------------------------------------------------------------------

        # Marvin左臂夹抓任务
        @self.ui.left_marvin_grip.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_grip(event)

        # Marvin左臂释放任务
        @self.ui.left_marvin_release.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_release(event)


        @self.ui.left_marvin_put_pose.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_put_pose(event)

        @self.ui.left_marvin_put_l.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_put_l(event)

        @self.ui.left_marvin_put_in.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_put_in(event)
            
        @self.ui.left_marvin_put_out.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_put_out(event)  


    #----- Marvin 相关任务处理 ----------------------------------------------------------
    def initialize_marvin(self):
        """初始化Marvin"""
        dcss=DCSS()
        cp = "src/robots_orchestra/driver/config/ccs_m6.MvKDCfg"
        if cp:
            ini_result = self.marvin_kine.load_config(config_path=cp)
            # print(f"ini_results:{ini_result}")
            if ini_result:
                self.marvin_kine.initial_kine(
                    robot_serial=0,
                    robot_type=ini_result["TYPE"][0],
                    dh=ini_result["DH"][0],
                    pnva=ini_result["PNVA"][0],
                    j67=ini_result["BD"][0],
                )
                ini_result2 = self.marvin_kine_2.load_config(config_path=cp)
                self.marvin_kine_2.initial_kine(
                    robot_serial=1,
                    robot_type=ini_result2["TYPE"][0],
                    dh=ini_result2["DH"][0],
                    pnva=ini_result2["PNVA"][0],
                    j67=ini_result2["BD"][0],
                )

        '''查验连接是否成功'''
        init = self.marvin_driver.connect('172.31.1.68')
        if init==0:
            print('failed:端口占用，连接失败!')
            return
        else:
            '''防总线通信异常,先清错'''
            time.sleep(0.5)
            self.marvin_driver.clear_set()
            self.marvin_driver.clear_error('A')
            self.marvin_driver.clear_error('B')
            self.marvin_driver.send_cmd()
            time.sleep(0.5)

            motion_tag = 0
            frame_update = None
            for i in range(5):
                sub_data = self.marvin_driver.subscribe(dcss)
                print(f"connect frames :{sub_data['outputs'][0]['frame_serial']}")
                if sub_data['outputs'][0]['frame_serial'] != 0 and frame_update != sub_data['outputs'][0]['frame_serial']:
                    motion_tag += 1
                    frame_update = sub_data['outputs'][0]['frame_serial']
                time.sleep(0.1)
            if motion_tag > 0:
                print('success:机器人连接成功!')
            else:
                print('failed:机器人连接失败!')
                return


        '''开启日志以便检查'''
        self.marvin_driver.log_switch('1') #全局日志开关
        self.marvin_driver.local_log_switch('1') # 主要日志


        '''清错'''
        self.marvin_driver.clear_set()
        self.marvin_driver.clear_error('A')
        self.marvin_driver.send_cmd()
        time.sleep( 0.5 )


        # # 初始化夹抓
        # result = self.marvin_driver.init_gripper(arm='A', is_full=True, channel=2)
        # if result:
        #     print(f"Marvin左臂夹爪初始化成功")
        # else:
        #     print(f"Marvin左臂夹爪初始化失败")

        sub_data = self.marvin_driver.subscribe(dcss)
        joint_data = self.get_marvin_joint_data( sub_data, 0 )
        print(f'joint_data: {joint_data}')

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
        # 直接调用session的load_scene方法，所有逻辑都在UserSession内部处理
        session.load_scene(self.marvin_kine, self.marvin_kine_2)

    def handle_clear_scene(self, session: UserSession, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理清除场景按钮事件"""
        client = session.client
        
        if not session.is_scene_loaded:
            return  # 如果场景未加载，不允许清除
    
        session.clear_scene() #清空加载的场景

    def handle_left_marvin_grip(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂夹抓按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_grip( self.marvin_driver )

    def handle_left_marvin_release(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂释放按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_release( self.marvin_driver )



    # ----------------------------------------------------------天机装配任务处理----------------------------------------------------------
    def handle_right_marvin_home(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机回到home按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.right_marvin_home( self.marvin_driver ) 

    def handle_right_marvin_first_capture(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第一个拍照按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.right_marvin_first_capture( self.marvin_driver )

    def handle_right_marvin_first_icp(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第一个icp配准按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.right_marvin_first_icp( self.marvin_driver, self.marvin_kine_2 )

    def handle_right_marvin_second_capture(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第二个拍照按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return  
        
        session = self.sessions[client]
        session.right_marvin_second_capture( self.marvin_driver )

    def handle_right_marvin_second_icp(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第二个icp配准按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.right_marvin_second_icp( self.marvin_driver, self.marvin_kine_2 )

    def handle_left_marvin_home(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂回到home按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_home( self.marvin_driver )

    

    def handle_left_marvin_pre_zhua_move(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂预抓取到预装配movel按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_pre_zhua_move( self.marvin_driver )
        
    def handle_left_marvin_pre_zhua_put_movel(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂预抓取到预装配movel按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_pre_zhua_put_movel( self.marvin_driver, self.marvin_kine )

    def handle_left_marvin_pre_zhua_to_put(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """实际下发执行从预抓取到预装配的movel"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_pre_zhua_to_put( self.marvin_driver )

    def handle_left_marvin_pre_zhua_to_put_reverse(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂回退到预抓取位置按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_pre_zhua_to_put_reverse( self.marvin_driver )

    def handle_left_marvin_zhua_pose(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂抓取姿态生成按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_zhua_pose( self.marvin_driver, self.marvin_kine )

    def handle_left_marvin_zhua_l(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂抓取movel按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_zhua_l( self.marvin_driver, self.marvin_kine )

    def handle_left_marvin_zhua_in(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂抓取进近按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_zhua_in( self.marvin_driver )

    def handle_left_marvin_zhua_out(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂抓取退出按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]

        session.left_marvin_zhua_out( self.marvin_driver )


    def handle_left_marvin_put_pose(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂放置姿态生成按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_put_pose( self.marvin_driver, self.marvin_kine )

    def handle_left_marvin_put_l(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂放置movel按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_put_l( self.marvin_driver, self.marvin_kine )

    def handle_left_marvin_put_in(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂放置进近按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_put_in( self.marvin_driver )

    def handle_left_marvin_put_out(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂放置退出按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_put_out( self.marvin_driver )

    def run(self):
        """运行控制器"""
        self.ui.run()