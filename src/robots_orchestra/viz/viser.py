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
        self.load_rolab()

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
            with self.server.gui.add_folder("框架移动任务", expand_by_default = False):
                self.abb_offline_planning = self.server.gui.add_button("ABB 框架移动规划", icon=viser.Icon.MOUSE)
                self.abb_view_simulation = self.server.gui.add_folder("查看仿真")  
                self.abb_execute = self.server.gui.add_button("任务执行", icon=viser.Icon.HAND_MOVE)

            with self.server.gui.add_folder("电焊线配置任务", expand_by_default = False):
                pass
                # self.marvin_go_station = self.server.gui.add_button("ABB 框架移动", icon=viser.Icon.MOUSE)
                # self.abb_view_simulation = self.server.gui.add_folder("查看仿真")  # 创建"查看仿真"文件夹，用于显示仿真界面


            with self.server.gui.add_folder("工件装配任务", expand_by_default = False):
                pass
                # self.marvin_go_home = self.server.gui.add_button("回到加工台", icon=viser.Icon.HAND_MOVE)
                # self.marvin_capture = self.server.gui.add_button("定位加持工件", icon=viser.Icon.CAPTURE)
                # self.marvin_show_capture_pcl = self.server.gui.add_checkbox("显示定位点云", initial_value=False)
                # self.marvin_capture_pose = self.server.gui.add_button("生成抓取姿态", icon=viser.Icon.CAPTURE)
                # self.marvin_show_capture_pose = self.server.gui.add_checkbox("显示抓取姿态", initial_value=False)

            
            with self.server.gui.add_folder("焊接打磨任务"):
                with self.server.gui.add_folder("Fanuc右臂焊接任务", expand_by_default = True):
                    with self.server.gui.add_folder("焊接小车移动任务", expand_by_default = False):
                        self.fanuc_car_move = self.server.gui.add_button("焊接小车规划", icon=viser.Icon.MOUSE)
                        self.fanuc_car_view_simulation = self.server.gui.add_folder("查看仿真")
                        self.fanuc_car_execute = self.server.gui.add_button("任务执行", icon=viser.Icon.HAND_MOVE)

                    with self.server.gui.add_folder("焊接定位任务", expand_by_default = False):
                        self.franka_locate = self.server.gui.add_button("焊接定位规划", icon=viser.Icon.MOUSE)
                        self.franka_view_simulation = self.server.gui.add_folder("查看仿真")
                        self.franka_capture = self.server.gui.add_button("执行拍照", icon=viser.Icon.CAMERA)
                        self.franka_capture_pcl = self.server.gui.add_folder("查看点云")


                    with self.server.gui.add_folder("焊接姿态生成", expand_by_default = False):
                        pass

                with self.server.gui.add_folder("Fanuc左臂打磨任务", expand_by_default = False):
                    pass

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
