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
        self.initialize_marvin()
        
        self.sessions: Dict[viser.ClientHandle, UserSession] = {}

        @self.ui.server.on_client_connect
        def _(client: viser.ClientHandle):
            self.handle_connect(client)
            self.set_default_camera(client)

        @self.ui.server.on_client_disconnect
        def _(client: viser.ClientHandle):
            self.handle_disconnect(client)

        # 全局按钮事件处理（从event中获取client信息）
        @self.ui.abb_offline_planning.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_abb_offline_planning(event)
        
        
        # 焊缝定位拍照
        @self.ui.franka_capture.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_franka_capture(event)

        # Marvin左臂拍照任务
        @self.ui.left_marvin_capture.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_capture(event)

        # Marvin左臂抓取任务
        @self.ui.left_marvin_crawl.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_crawl(event)

        # Marvin左臂移动到抓取姿态任务
        @self.ui.left_marvin_moving.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_moving(event)

        # Marvin左臂回到预夹取位置任务
        @self.ui.left_marvin_go_before.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_go_before(event)

        # Marvin左臂夹抓任务
        @self.ui.left_marvin_grip.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_grip(event)

        # Marvin左臂释放任务
        @self.ui.left_marvin_release.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_release(event)

        # Marvin左臂回到home任务
        @self.ui.left_marvin_go_home.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_go_home(event)

        # Marvin左臂第一次装配home任务
        @self.ui.left_marvin_first_home.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_first_home(event)

        # Marvin左臂第一次装配预夹取位置任务
        @self.ui.left_marvin_first_pre.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_first_pre(event)

        # Marvin左臂第一次装配姿态生成任务
        @self.ui.left_marvin_first_pose.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_first_pose(event)

        # Marvin左臂第一次装配移动到装配位置任务
        @self.ui.left_marvin_first_put.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_first_put(event)

        # Marvin左臂第一次装配力控任务
        @self.ui.left_marvin_first_force.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_first_force(event)


        # Marvin左臂第一次装配进经状态任务
        @self.ui.left_marvin_first_jing.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_first_jing(event)

        # Marvin左臂第一次装配插入任务
        @self.ui.left_marvin_first_cha.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_first_cha(event)

        # Marvin左臂第一次装配退出任务
        @self.ui.left_marvin_first_out.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_first_out(event)

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


        #初始化夹抓
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
        session.load_scene()

    def handle_clear_scene(self, session: UserSession, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理清除场景按钮事件"""
        client = session.client
        
        if not session.is_scene_loaded:
            return  # 如果场景未加载，不允许清除
    
        session.clear_scene() #清空加载的场景

    def handle_abb_offline_planning(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理ABB框架移动按钮事件（全局按钮，从event中获取触发事件的客户端）"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.abb_offline_planning()


    def handle_franka_capture(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理焊接定位拍照按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.franka_capture()

    def handle_left_marvin_capture(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂拍照按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_capture()

    def handle_left_marvin_crawl(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂抓取按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_crawl()

    def handle_left_marvin_moving(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂移动到抓取姿态按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        dcss=DCSS()
        joint_data = self.get_marvin_joint_data( self.marvin_driver.subscribe(dcss), 0 )
        session = self.sessions[client]
        session.left_marvin_moving( self.marvin_driver, self.marvin_kine, joint_data)

    def handle_left_marvin_go_before(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂回到预夹取位置按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_go_before( self.marvin_driver, self.marvin_kine )

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
    
    def handle_left_marvin_go_home(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂回到home按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_go_home( self.marvin_driver )

    def handle_left_marvin_first_home(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂第一次装配home按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_first_home( self.marvin_driver )

    def handle_left_marvin_first_pre(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂第一次装配预夹取位置按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_first_pre( self.marvin_driver )

    def handle_left_marvin_first_pose(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂第一次装配姿态生成按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_first_pose( self.marvin_driver )

    def handle_left_marvin_first_put(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂第一次装配移动到装配位置按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        dcss=DCSS()
        joint_data = self.get_marvin_joint_data( self.marvin_driver.subscribe(dcss), 0 )
        session.left_marvin_first_put( self.marvin_driver, self.marvin_kine, joint_data )

    def handle_left_marvin_first_force(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂第一次装配力控按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]

        dcss=DCSS()
        joint_data = self.get_marvin_joint_data( self.marvin_driver.subscribe(dcss), 0 )
        session.left_marvin_first_force( self.marvin_driver, self.marvin_kine, joint_data )

    def handle_left_marvin_first_jing(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂第一次装配进经状态按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_first_jing( self.marvin_driver )


    def handle_left_marvin_first_cha(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂第一次装配插入按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_first_cha( self.marvin_driver )

    def handle_left_marvin_first_out(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂第一次装配退出按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_first_out( self.marvin_driver )

    def run(self):
        """运行控制器"""
        self.ui.run()