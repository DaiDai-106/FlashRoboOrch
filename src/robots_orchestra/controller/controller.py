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
    
        @self.ui.right_marvin_second_capture.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_right_marvin_second_capture(event)

        @self.ui.left_marvin_home.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_home(event)

        # 分界线，上面是天机装配任务------------------------------------------
        @self.ui.a_pre_zhua_state.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_a_pre_zhua_state(event)
        @self.ui.a_pre_put_state.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_a_pre_put_state(event)
        @self.ui.b_pre_zhua_state.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_b_pre_zhua_state(event)
        @self.ui.b_pre_put_state.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_b_pre_put_state(event)
        @self.ui.a_zhua_sim.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_a_zhua_sim(event)
        @self.ui.a_put_sim.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_a_put_sim(event)
        @self.ui.a_zhua_real_do.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_a_zhua_real_do(event)
        @self.ui.a_put_real_do.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_a_put_real_do(event)
        @self.ui.a_zhua_real_do_reverse.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_a_zhua_real_do_reverse(event)
        @self.ui.a_put_real_do_reverse.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_a_put_real_do_reverse(event)
        @self.ui.a_whole_process.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_a_whole_process(event)
        @self.ui.b_zhua_sim.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_b_zhua_sim(event)
        @self.ui.b_put_sim.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_b_put_sim(event)
        @self.ui.b_zhua_real_do.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_b_zhua_real_do(event)
        @self.ui.b_put_real_do.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_b_put_real_do(event)
        @self.ui.b_zhua_real_do_reverse.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_b_zhua_real_do_reverse(event)
        @self.ui.b_put_real_do_reverse.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_b_put_real_do_reverse(event)
        @self.ui.b_put_insert.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_b_put_insert(event)
        @self.ui.b_put_insert_reverse.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_b_put_insert_reverse(event)
        @self.ui.b_whole_process.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_b_whole_process(event)
        
        #-------------------------------------------------------------------------------------------------

        # Marvin左臂夹抓任务
        @self.ui.left_marvin_grip.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_grip(event)

        # Marvin左臂释放任务
        @self.ui.left_marvin_release.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_left_marvin_release(event)  

        # 天机力控插入任务 ------------------------------------------
        @self.ui.first_insert_capture.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_first_insert_capture(event)

        @self.ui.first_insert_icp.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_first_insert_icp(event)

        @self.ui.first_insert_get_object.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_first_insert_get_object(event)

        @self.ui.first_insert_back_object.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_first_insert_back_object(event)

        @self.ui.first_insert_sim.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_first_insert_sim(event)

        @self.ui.first_insert_real_do.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_first_insert_real_do(event)

        @self.ui.first_insert_real_do_inset.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_first_insert_real_do_inset(event)

        @self.ui.first_insert_real_do_inset_reverse.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_first_insert_real_do_inset_reverse(event)


        # 天机的第二次插入的任务-------------------------------------------------------------------
        @self.ui.second_insert_capture.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_second_insert_capture(event)

        @self.ui.second_insert_icp.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_second_insert_icp(event)

        @self.ui.second_insert_get_object.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_second_insert_get_object(event)

        @self.ui.second_insert_back_object.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_second_insert_back_object(event)

        @self.ui.second_insert_sim.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_second_insert_sim(event)

        @self.ui.second_insert_real_do.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_second_insert_real_do(event)

        @self.ui.second_insert_real_do_inset.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_second_insert_real_do_inset(event)

        @self.ui.second_insert_real_do_inset_reverse.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_second_insert_real_do_inset_reverse(event)

        # Marvin基础功能
        @self.ui.marvin_stop.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_marvin_stop(event)
        @self.ui.marvin_clear_error.on_click
        def _(event: viser.GuiEvent[viser.GuiButtonHandle]):
            self.handle_marvin_clear_error(event)

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

    def handle_right_marvin_second_capture(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第二个拍照按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return  
        
        session = self.sessions[client]
        session.right_marvin_second_capture( self.marvin_driver )

    def handle_left_marvin_home(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin左臂回到home按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.left_marvin_home( self.marvin_driver )

    
    def handle_a_pre_zhua_state(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理A预抓状态按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.a_pre_zhua_state( self.marvin_driver )

    def handle_a_pre_put_state(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理A预放状态按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.a_pre_put_state( self.marvin_driver )

    def handle_b_pre_zhua_state(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理B预抓状态按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.b_pre_zhua_state( self.marvin_driver )

    def handle_b_pre_put_state(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理B预放状态按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.b_pre_put_state( self.marvin_driver )
    
    def handle_a_zhua_sim(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理A抓取作用补偿后仿真按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.a_zhua_sim( self.marvin_driver )

    def handle_a_put_sim(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理A放置作用补偿后仿真按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.a_put_sim( self.marvin_driver )

    def handle_a_zhua_real_do(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理A抓取实际执行按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return

        session = self.sessions[client]
        session.a_zhua_real_do( self.marvin_driver )

    def handle_a_put_real_do(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理A放置实际执行按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return

        session = self.sessions[client]
        session.a_put_real_do( self.marvin_driver )

    def handle_a_zhua_real_do_reverse(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理A抓取实际执行回退按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.a_zhua_real_do_reverse( self.marvin_driver )

    def handle_a_put_real_do_reverse(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理A放置实际执行回退按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.a_put_real_do_reverse( self.marvin_driver )

    def handle_a_whole_process(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理A实际执行完整过程按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.a_whole_process( self.marvin_driver )

    def handle_b_zhua_sim(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理B抓取作用补偿后仿真按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.b_zhua_sim( self.marvin_driver )

    def handle_b_put_sim(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理B放置作用补偿后仿真按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.b_put_sim( self.marvin_driver )

    def handle_b_put_real_do(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理B放置实际执行按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.b_put_real_do( self.marvin_driver )     

    def handle_b_put_real_do_reverse(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理B放置实际执行回退按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.b_put_real_do_reverse( self.marvin_driver )
        
    def handle_b_put_insert(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理B放置插入按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.b_put_insert( self.marvin_driver )

    def handle_b_put_insert_reverse(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理B放置插入回退按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.b_put_insert_reverse( self.marvin_driver )

    def handle_b_whole_process(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理B实际执行完整过程按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.b_whole_process( self.marvin_driver )   
    #-----------------------------------------------纯分割线-----------------------------------------------


    # 分界线，下面是天机力控插入任务------------------------------------------
    def handle_first_insert_capture(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第一次力控插入拍照按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client] 
        session.first_insert_capture( self.marvin_driver )

    def handle_first_insert_icp(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第一次力控插入icp配准按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.first_insert_icp( self.marvin_driver, self.marvin_kine_2 ) 

    def handle_first_insert_get_object(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第一次力控插入获取物体按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.first_insert_get_object( self.marvin_driver )

    def handle_first_insert_back_object(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第一次力控插入回退物体按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.first_insert_back_object( self.marvin_driver )

    def handle_first_insert_sim(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第一次力控插入仿真按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.first_insert_sim( self.marvin_driver, self.marvin_driver )

    def handle_first_insert_real_do(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第一次力控插入实际执行按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        session = self.sessions[client]
        session.first_insert_real_do( self.marvin_driver )

    def handle_first_insert_real_do_inset(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第一次力控插入实际执行插入按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        dcss = DCSS()
        sub_data = self.marvin_driver.subscribe(dcss)
        joint =self.get_marvin_joint_data( sub_data, 0)
        session = self.sessions[client]
        session.first_insert_real_do_inset( self.marvin_driver, joint )

    def handle_first_insert_real_do_inset_reverse(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第一次力控插入实际执行插入回退按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.first_insert_real_do_inset_reverse( self.marvin_driver )


    # 天机的第二次插入的任务-------------------------------------------------------------------

    def handle_second_insert_capture(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第二次力控插入拍照按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.second_inseet_capture( self.marvin_driver )

    def handle_second_insert_icp(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第二次力控插入icp配准按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.second_insert_icp( self.marvin_driver, self.marvin_kine_2 )

    def handle_second_insert_get_object(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第二次力控插入获取物体按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.second_insert_get_object( self.marvin_driver )


    def handle_second_insert_back_object(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第二次力控插入回退物体按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.second_insert_back_object( self.marvin_driver )

    def handle_second_insert_sim(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第二次力控插入仿真按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.second_insert_sim( self.marvin_driver, self.marvin_driver )

    def handle_second_insert_real_do(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第二次力控插入实际执行按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.second_insert_real_do( self.marvin_driver )

    def handle_second_insert_real_do_inset(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第二次力控插入实际执行插入按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:   
            print(f"警告: 无法找到触发事件的客户端")
            return
        
        dcss = DCSS()
        sub_data = self.marvin_driver.subscribe(dcss)
        joint =self.get_marvin_joint_data( sub_data, 0)
        session = self.sessions[client]
        session.second_insert_real_do_inset( self.marvin_driver, joint )

    def handle_second_insert_real_do_inset_reverse(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理天机第二次力控插入实际执行插入回退按钮事件"""
        client = event.client
        if client is None or client not in self.sessions:
            print(f"警告: 无法找到触发事件的客户端")
            return
        session = self.sessions[client]
        session.second_insert_real_do_inset_reverse( self.marvin_driver )

    # Marvin基础功能
    def handle_marvin_stop(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin急停按钮事件"""
        self.marvin_driver.soft_stop("AB")
        time.sleep(0.5)

    def handle_marvin_clear_error(self, event: viser.GuiEvent[viser.GuiButtonHandle]):
        """处理Marvin清错按钮事件"""
        time.sleep(0.5)
        self.marvin_driver.clear_set()
        self.marvin_driver.clear_error('A')
        self.marvin_driver.clear_error('B')
        self.marvin_driver.send_cmd()
        time.sleep(0.5)

    def run(self):
        """运行控制器"""
        self.ui.run()