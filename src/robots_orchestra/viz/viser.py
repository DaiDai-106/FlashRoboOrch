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
            with self.server.gui.add_folder("天机装配任务", expand_by_default = False):
                with self.server.gui.add_folder("右臂感知任务"):
                    self.right_marvin_home = self.server.gui.add_button("移动到拍照的home状态", icon=viser.Icon.HAND_MOVE)
                    self.right_marvin_first_capture = self.server.gui.add_button("移动到右手的第一个拍照状态", icon=viser.Icon.CAMERA)
                    self.right_marvin_first_icp = self.server.gui.add_button("第一个模型的icp配准", icon=viser.Icon.CAPTURE)
                    self.right_marvin_second_capture = self.server.gui.add_button("移动到右手的第二个拍照状态", icon=viser.Icon.CAMERA)
                    self.right_marvin_second_icp = self.server.gui.add_button("第二个模型的icp配准", icon=viser.Icon.CAPTURE)
                with self.server.gui.add_folder("左臂抓取任务"):
                    self.left_marvin_home = self.server.gui.add_button("移动到左手的home状态", icon=viser.Icon.HAND_MOVE)
                    self.left_marvin_pre_zhua_move = self.server.gui.add_button("移动到左手的预抓取位置", icon=viser.Icon.HAND_MOVE)  
                    with self.server.gui.add_folder("左臂预抓取到预装配movel"):
                        self.left_marvin_pre_zhua_put_movel = self.server.gui.add_button("求解moveL", icon=viser.Icon.HAND_MOVE)
                        self.left_marvin_pre_zhua_put_movel_view_simulation = self.server.gui.add_folder("查看仿真")  # 这是个Movel
                        self.left_marvin_pre_zhua_to_put = self.server.gui.add_button("实际执行移动到左手的预装配位置", icon=viser.Icon.HAND_MOVE)
                        self.left_marvin_pre_zhua_to_put_reverse = self.server.gui.add_button("回退到预抓取位置", icon=viser.Icon.HAND_MOVE)

                    with self.server.gui.add_folder("左臂抓取进近"):
                        self.left_marvin_zhua_pose =  self.server.gui.add_button("生成抓的姿态", icon=viser.Icon.HAND_MOVE)
                        self.left_marvin_zhua_l =  self.server.gui.add_button("求解移动到抓取movel", icon=viser.Icon.HAND_MOVE)
                        self.left_marvin_zhua_view_simulation = self.server.gui.add_folder("查看仿真")
                        self.left_marvin_zhua_in =  self.server.gui.add_button("实机执行进入抓取", icon=viser.Icon.HAND_MOVE)
                        self.left_marvin_zhua_out =  self.server.gui.add_button("实机执行回预抓取", icon=viser.Icon.HAND_MOVE)

                    with self.server.gui.add_folder("左臂放置进近"):
                        self.left_marvin_put_pose =  self.server.gui.add_button("生成放的姿态", icon=viser.Icon.HAND_MOVE)
                        self.left_marvin_put_l =  self.server.gui.add_button("求解移动到放置的movel", icon=viser.Icon.HAND_MOVE)
                        self.left_marvin_put_view_simulation = self.server.gui.add_folder("查看仿真")
                        self.left_marvin_put_in =  self.server.gui.add_button("实机执行进入放置", icon=viser.Icon.HAND_MOVE)
                        self.left_marvin_put_out =  self.server.gui.add_button("实机执行回预放置", icon=viser.Icon.HAND_MOVE)


             
                    
            with self.server.gui.add_folder("Marvin左臂夹抓控制", expand_by_default = True):
                    self.left_marvin_grip = self.server.gui.add_button("夹抓", icon=viser.Icon.HAND_MOVE)
                    self.left_marvin_release = self.server.gui.add_button("释放", icon=viser.Icon.HAND_MOVE)

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
