import viser
import time
import ampl
from robots_orchestra import SCENE_DIR

# 这里只负责 UI 的设计
class ViserUI:
    def __init__(self, title: str = "Robots Orchestra", port: int = 8080):
        self.server = viser.ViserServer( port=port, label=title)
        self.initialize_ui()  # 初始化部分固定的 GUI 设计

         # 这里对各个子任务进行系统性的梳理
        self.initialize_process()
        # self.load_rolab()  # 暂时先不加载了

    def initialize_ui(self):
        # 添加默认网格 (grid) - XY 平面，适合机器人场景
        self.server.scene.add_grid(
            name="/world",
            width=14.0,
            height=14.0,
            plane="xy",  # XY 平面，Z 轴向上
            cell_size=0.5,
            section_size=1.0,
            section_color=[0, 0, 0],
            cell_color=[128, 128, 128],
            cell_thickness=0.5,
            infinite_grid=False,  # 无限网格
        )
        
        # 在原点 (0, 0, 0) 显示坐标系
        self.server.scene.add_frame(
            name="/world/origin",
            show_axes=True,
            axes_length=1.0,  # 坐标轴长度
            axes_radius=0.02,  # 坐标轴半径
            position=(0.0, 0.0, 0.0)  # 原点位置
        )

        # 创建"场景加载"文件夹并保存引用（现在只作为容器，按钮将在UserSession中为每个用户创建）
        self.scene_folder = self.server.gui.add_folder("场景加载")
        with self.scene_folder:
            self.robot_drag_folder = self.server.gui.add_folder("机器人拖动") # 创建"机器人拖动"文件夹，用于显示每个机器人的关节slider

    """长任务执行单元, 目前只是测试， 后续应该会尽可能的罗列出所有子任务"""
    def initialize_process(self):
        with self.server.gui.add_folder("长任务执行单元"):
            self.right_marvin_home = self.server.gui.add_button("回右臂home状态", icon=viser.Icon.HAND_MOVE)
            self.left_marvin_home = self.server.gui.add_button("回左臂home状态", icon=viser.Icon.HAND_MOVE)
            with self.server.gui.add_folder("天机装配任务", expand_by_default = False):
                with self.server.gui.add_folder("右臂感知任务", expand_by_default = False):
                    self.right_marvin_first_capture = self.server.gui.add_button("移动到右手的第一个拍照状态", icon=viser.Icon.CAMERA)
                    self.right_marvin_second_capture = self.server.gui.add_button("移动到右手的第二个拍照状态", icon=viser.Icon.CAMERA)
                with self.server.gui.add_folder("左臂第一次装配任务", expand_by_default = False):
                    self.a_pre_zhua_state = self.server.gui.add_button("A预抓状态", icon=viser.Icon.HAND_MOVE)
                    self.a_pre_put_state = self.server.gui.add_button("A预放状态", icon=viser.Icon.HAND_MOVE)
                    with self.server.gui.add_folder("抓取作用补偿后仿真", expand_by_default = False): 
                        self.a_zhua_compensation_x = self.server.gui.add_number("全局X (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                        self.a_zhua_compensation_y = self.server.gui.add_number("全局Y (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                        self.a_zhua_compensation_z = self.server.gui.add_number("全局Z (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                        self.a_zhua_compensation_a = self.server.gui.add_number("TCP的A (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                        self.a_zhua_compensation_b = self.server.gui.add_number("TCP的B (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                        self.a_zhua_compensation_c = self.server.gui.add_number("TCP的C (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                        self.a_zhua_compensation_flange_x = self.server.gui.add_number("法兰的X)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                        self.a_zhua_compensation_flange_y = self.server.gui.add_number("法兰的Y)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                        self.a_zhua_compensation_flange_z = self.server.gui.add_number("法兰的Z)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                        self.a_zhua_sim = self.server.gui.add_button("使用补偿进行仿真", icon=viser.Icon.HAND_MOVE)
                        self.a_zhua_view_simulation = self.server.gui.add_folder("查看仿真")
                        self.a_zhua_real_do = self.server.gui.add_button("实际执行")
                        self.a_zhua_real_do_reverse = self.server.gui.add_button("回退")
                    with self.server.gui.add_folder("放置作用补偿后仿真", expand_by_default = False): 
                        self.a_put_compensation_x = self.server.gui.add_number("全局X (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                        self.a_put_compensation_y = self.server.gui.add_number("全局Y (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                        self.a_put_compensation_z = self.server.gui.add_number("全局Z (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                        self.a_put_compensation_a = self.server.gui.add_number("TCP的A (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                        self.a_put_compensation_b = self.server.gui.add_number("TCP的B (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                        self.a_put_compensation_c = self.server.gui.add_number("TCP的C (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                        self.a_put_compensation_flange_x = self.server.gui.add_number("法兰的X)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                        self.a_put_compensation_flange_y = self.server.gui.add_number("法兰的Y)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                        self.a_put_compensation_flange_z = self.server.gui.add_number("法兰的Z)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                        self.a_put_sim = self.server.gui.add_button("使用补偿进行仿真", icon=viser.Icon.HAND_MOVE)
                        self.a_put_view_simulation = self.server.gui.add_folder("查看仿真")
                        self.a_put_real_do = self.server.gui.add_button("实际执行")
                        self.a_put_real_do_reverse = self.server.gui.add_button("回退")
                    self.a_whole_process = self.server.gui.add_button("实际执行完整过程", icon=viser.Icon.HAND_MOVE)
                    
                with self.server.gui.add_folder("左臂第二次装配任务", expand_by_default = False):
                    self.b_pre_zhua_state = self.server.gui.add_button("B预抓状态", icon=viser.Icon.HAND_MOVE)
                    self.b_pre_put_state = self.server.gui.add_button("B预放状态", icon=viser.Icon.HAND_MOVE)
                    with self.server.gui.add_folder("抓取作用补偿后仿真", expand_by_default = False): 
                        self.b_zhua_compensation_x = self.server.gui.add_number("全局X (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                        self.b_zhua_compensation_y = self.server.gui.add_number("全局Y (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                        self.b_zhua_compensation_z = self.server.gui.add_number("全局Z (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                        self.b_zhua_compensation_a = self.server.gui.add_number("TCP的A (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                        self.b_zhua_compensation_b = self.server.gui.add_number("TCP的B (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                        self.b_zhua_compensation_c = self.server.gui.add_number("TCP的C (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                        self.b_zhua_compensation_flange_x = self.server.gui.add_number("法兰的X)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                        self.b_zhua_compensation_flange_y = self.server.gui.add_number("法兰的Y)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                        self.b_zhua_compensation_flange_z = self.server.gui.add_number("法兰的Z)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                        self.b_zhua_sim = self.server.gui.add_button("使用补偿进行仿真", icon=viser.Icon.HAND_MOVE)
                        self.b_zhua_view_simulation = self.server.gui.add_folder("查看仿真")
                        self.b_zhua_real_do = self.server.gui.add_button("实际执行")
                        self.b_zhua_real_do_reverse = self.server.gui.add_button("回退")
                    with self.server.gui.add_folder("放置作用补偿后仿真", expand_by_default = False): 
                        self.b_put_compensation_x = self.server.gui.add_number("全局X (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                        self.b_put_compensation_y = self.server.gui.add_number("全局Y (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                        self.b_put_compensation_z = self.server.gui.add_number("全局Z (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                        self.b_put_compensation_a = self.server.gui.add_number("TCP的A (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                        self.b_put_compensation_b = self.server.gui.add_number("TCP的B (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                        self.b_put_compensation_c = self.server.gui.add_number("TCP的C (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                        self.b_put_compensation_flange_x = self.server.gui.add_number("法兰的X)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                        self.b_put_compensation_flange_y = self.server.gui.add_number("法兰的Y)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                        self.b_put_compensation_flange_z = self.server.gui.add_number("法兰的Z)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                        self.b_put_sim = self.server.gui.add_button("使用补偿进行仿真", icon=viser.Icon.HAND_MOVE)
                        self.b_put_view_simulation = self.server.gui.add_folder("查看仿真")
                        self.b_put_real_do = self.server.gui.add_button("实际执行")
                        self.b_put_real_do_reverse = self.server.gui.add_button("回退")
                    # 这里会多一个插入的过程
                    self.b_put_insert = self.server.gui.add_button("插入", icon=viser.Icon.HAND_MOVE)
                    self.b_put_insert_reverse = self.server.gui.add_button("插入回退", icon=viser.Icon.HAND_MOVE)
                    self.b_whole_process = self.server.gui.add_button("实际执行完整过程", icon=viser.Icon.HAND_MOVE)


            # 第一次力控的任务
            with self.server.gui.add_folder("第一次力控插入任务", expand_by_default = False):
                self.first_insert_capture = self.server.gui.add_button("右臂拍照", icon=viser.Icon.HAND_MOVE)
                self.first_insert_icp = self.server.gui.add_button("icp配准", icon=viser.Icon.HAND_MOVE) # 这个相机就会回home了
                self.first_insert_get_object = self.server.gui.add_button("获取物体", icon=viser.Icon.HAND_MOVE)
                self.first_insert_back_object = self.server.gui.add_button("回退物体", icon=viser.Icon.HAND_MOVE)

                # 补偿值输入
                with self.server.gui.add_folder("作用补偿后仿真", expand_by_default = False):
                    self.first_insert_compensation_x = self.server.gui.add_number("全局X (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                    self.first_insert_compensation_y = self.server.gui.add_number("Y (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                    self.first_insert_compensation_z = self.server.gui.add_number("全局Z (mm)", min=-100.0, max=100.0, step=0.1, initial_value=0.0)
                    self.first_insert_compensation_a = self.server.gui.add_number("TCP的A (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                    self.first_insert_compensation_b = self.server.gui.add_number("TCP的B (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                    self.first_insert_compensation_c = self.server.gui.add_number("TCP的C (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                    self.first_insert_compensation_wp_x = self.server.gui.add_number("工件坐标系的X)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                    self.first_insert_compensation_wp_y = self.server.gui.add_number("工件坐标系的Y)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                    self.first_insert_compensation_wp_z = self.server.gui.add_number("工件坐标系的Z)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)  

                    self.first_insert_sim = self.server.gui.add_button("使用补偿进行仿真", icon=viser.Icon.HAND_MOVE)
                    self.first_insert_view_simulation = self.server.gui.add_folder("查看仿真")
                    self.first_insert_real_do = self.server.gui.add_button("实际执行")
                    self.first_insert_real_do_inset = self.server.gui.add_button("插入")
                    self.first_insert_real_do_inset_reverse = self.server.gui.add_button("插入回退")


            # 第二次的力控任务
            with self.server.gui.add_folder("第二次力控插入任务", expand_by_default = False):
                self.second_insert_capture = self.server.gui.add_button("右臂拍照", icon=viser.Icon.HAND_MOVE)
                self.second_insert_icp = self.server.gui.add_button("icp配准", icon=viser.Icon.HAND_MOVE) # 这个相机就会回home了
                self.second_insert_get_object = self.server.gui.add_button("获取物体", icon=viser.Icon.HAND_MOVE)
                self.second_insert_back_object = self.server.gui.add_button("回退物体", icon=viser.Icon.HAND_MOVE)

                # 补偿值输入
                with self.server.gui.add_folder("作用补偿后仿真", expand_by_default = False):
                    self.second_insert_compensation_x = self.server.gui.add_number("全局X (mm)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                    self.second_insert_compensation_y = self.server.gui.add_number("全局Y (mm)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                    self.second_insert_compensation_z = self.server.gui.add_number("全局Z (mm)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                    self.second_insert_compensation_a = self.server.gui.add_number("TCP的A (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                    self.second_insert_compensation_b = self.server.gui.add_number("TCP的B (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                    self.second_insert_compensation_c = self.server.gui.add_number("TCP的C (度)", min=-180.0, max=180.0, step=0.1, initial_value=0.0)
                    self.second_insert_compensation_wp_x = self.server.gui.add_number("工件坐标系的X)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                    self.second_insert_compensation_wp_y = self.server.gui.add_number("工件坐标系的Y)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)
                    self.second_insert_compensation_wp_z = self.server.gui.add_number("工件坐标系的Z)", min=-500.0, max=500.0, step=0.1, initial_value=0.0)   


                    self.second_insert_sim = self.server.gui.add_button("使用补偿进行仿真", icon=viser.Icon.HAND_MOVE)
                    self.second_insert_view_simulation = self.server.gui.add_folder("查看仿真")
                    self.second_insert_real_do = self.server.gui.add_button("实际执行")
                    self.second_insert_real_do_inset = self.server.gui.add_button("插入")
                    self.second_insert_real_do_inset_reverse = self.server.gui.add_button("插入回退")
                    
        with self.server.gui.add_folder("Marvin左臂夹抓控制", expand_by_default = True):
            self.left_marvin_grip = self.server.gui.add_button("夹抓", icon=viser.Icon.HAND_MOVE)
            self.left_marvin_release = self.server.gui.add_button("释放", icon=viser.Icon.HAND_MOVE)


        with self.server.gui.add_folder("Marvin的基础功能", expand_by_default = True):
            self.marvin_stop = self.server.gui.add_button("急停", icon=viser.Icon.HAND_MOVE)
            self.marvin_clear_error = self.server.gui.add_button("清错", icon=viser.Icon.HAND_MOVE)

        # 这里主要包含

    def load_rolab(self):
        """加载并显示完整的实验室点云模型"""
        try:
            # 构建点云文件路径
            ply_path = SCENE_DIR / "rolab" / "rolap_under_abb.ply"
            pcd_v, pcd_c, _ = ampl.read_pointcloud(ply_path)
            pcd_c = pcd_c[:, ::-1]
            
            # 将点云添加到viser场景
            self.server.scene.add_point_cloud(
                name="/world/origin/rolab_world",
                points=pcd_v,
                colors=pcd_c,
                point_size=0.015,  # 点的大小
            )

            print(f"成功加载实验室点云: {ply_path}")
        except Exception as e:
            print(f"加载实验室点云时出错: {e}")


    @staticmethod
    def run():
        while True:
            time.sleep(0.1)
