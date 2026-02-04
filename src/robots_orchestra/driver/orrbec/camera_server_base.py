import json
import logging
import sys
import threading
import time
from enum import Enum
from typing import Optional

import fastapi
import numpy as np
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from PIL import Image
import io
import cv2

from Chin.core.TaskUtil import TaskUtil

# save_path = "/home/zx/Downloads/service_viser_TJ/temp/"
class ImageRequest(BaseModel):
    task_id: str
    folder: str = ""


class CameraServer:
    def __init__(self, app, task, streaming=True, frame_rate=1, resolution=(1280, 960)):
        self.camera_param = None

        self.task: Optional[TaskUtil] = task

        self.app: Optional[fastapi.FastAPI] = app

        self.config: Optional[dict] = None

        self.calibration_mode = False

        # 存储最新的帧数据
        self.latest_color_frame: Optional[np.array] = None
        self.latest_depth_frame: Optional[np.array] = None
        self.latest_color_recv_time: float = 0
        self.latest_depth_recv_time: float = 0

        self.frame_rate = frame_rate

        self.streaming = streaming

        self.resolution = resolution

        self.register_routes()


    def re_init(self, app, task, streaming=True, frame_rate=1, resolution=(1280, 960)):
        self.camera_param = None

        self.task: Optional[TaskUtil] = task

        self.app: Optional[fastapi.FastAPI] = app

        self.config: Optional[dict] = None

        self.calibration_mode = False

        # 存储最新的帧数据
        self.latest_color_frame: Optional[np.array] = None
        self.latest_depth_frame: Optional[np.array] = None
        self.latest_color_recv_time: float = 0
        self.latest_depth_recv_time: float = 0

        self.frame_rate = frame_rate

        self.streaming = streaming

        self.resolution = resolution

        self.register_routes()

    @property
    def width(self):
        return self.resolution[0]

    @property
    def height(self):
        return self.resolution[1]

    def get_image(self, depth=False):
        time_diff = time.time() - self.latest_color_recv_time
        frame = self.latest_color_frame
        color_data = cv2.cvtColor( frame, cv2.COLOR_BGR2RGB)
        # cv2.imwrite(save_path+'current_color.png', color_data)

        if depth:
            if self.calibration_mode:
                raise Exception("Depth is not supported in calibration mode ")

            time_diff = time.time() - self.latest_depth_recv_time
            frame = self.latest_depth_frame

            depth_data = frame.astype(np.float32) 
            # np.save(save_path+'current_depth.npy', depth_data)

        if time_diff > 2.0 / self.frame_rate:
            # logging.error("Latest frame is captured by 2 cycle ago.")
            # return None
            print("Latest frame is captured by 2 cycle ago.")
            print(time_diff)

            # self.re_init()



        # 将 NumPy 数组转换为 PIL 图像
        pil_image = Image.fromarray(frame)

        # 将 PIL 图像转换为字节流
        img_byte_arr = io.BytesIO()
        img_byte_arr.name = ("depth" if depth else "color") + ".png"
        pil_image.save(img_byte_arr, format='PNG')
        img_byte_arr.seek(0)

        return img_byte_arr

    def upload_intrinsic(self, task_id):
        intrinsic_bytes = io.BytesIO(json.dumps(self.camera_param).encode())
        intrinsic_bytes.name = "intrinsic.json"
        public_url = self.task.upload_fileobj(task_id, intrinsic_bytes, "original")

        return public_url

    def calibrationMode(self, on: bool = True):
        pass

    def capture(self):
        pass

    def initialize(self):
        pass

    def start_streaming(self):
        pass

    def stop_streaming(self):
        pass

    def register_routes(self):

        @self.app.get("/color")
        def get_color_frame(task_id: str, folder: str = ""):
            try:
                if not self.streaming:
                    self.capture()

                logging.info(f"upload color frame to {task_id}")

                # 将 PIL 图像转换为字节流
                img_byte_arr = self.get_image(False)

                if img_byte_arr is None:
                    # 返回图像数据
                    return JSONResponse(status_code=200,
                        content={"message": "Didn't receive latest frame.", "code": "7"})

                public_url = self.task.upload_fileobj(task_id, img_byte_arr, f"original/{folder}")

                # 返回图像数据
                return JSONResponse(status_code=200,
                    content={"message": "Upload task image successfully",
                             "task_id": task_id,
                             "color_url": public_url,
                             "code": 0,
                             "intrinsic": self.upload_intrinsic(task_id)})
            except Exception as e:
                return JSONResponse(status_code=400,
                    content={"message": f"Didn't receive latest frame with error: {e}.", "code": "8"})

        @self.app.post("/color")
        def get_color(params: ImageRequest):
            task_id = params.task_id

            return get_color_frame(task_id, params.folder)

        @self.app.get("/depth")
        def get_depht_frame(task_id: str, folder: str = ""):
            try:
                if not self.streaming:
                    self.capture()

                logging.info(f"upload depth frame to {task_id}")
                # 将 PIL 图像转换为字节流
                img_byte_arr = self.get_image(True)

                if img_byte_arr is None:
                    # 返回图像数据
                    return JSONResponse(status_code=200,
                        content={"message": "Didn't receive latest frame.", "code": "7"})

                public_url = self.task.upload_fileobj(task_id, img_byte_arr, f"original/{folder}")

                # 返回图像数据
                return JSONResponse(status_code=200,
                    content={"message": "Upload task image successfully",
                             "task_id": task_id,
                             "depth_url": public_url,
                             "code": 0,
                             "intrinsic": self.upload_intrinsic(task_id)})
            except Exception as e:
                return JSONResponse(status_code=200,
                    content={"message": f"Didn't receive latest frame with error: {e}.", "code": "8"})

        @self.app.post("/depth")
        def get_depth(params: ImageRequest):
            task_id = params.task_id

            return get_depht_frame(task_id, params.folder)

        @self.app.get("/image")
        def get_frame(task_id: str, folder: str = ""):
            try:
                if not self.streaming:
                    self.capture()

                logging.info(f"upload color/depth frame to {task_id}")
                color_byte_arr = self.get_image(False)
                depth_byte_arr = self.get_image(True)

                if color_byte_arr is None or depth_byte_arr is None:
                    # 返回图像数据
                    return JSONResponse(status_code=200,
                        content={"message": "Didn't receive latest frame.", "code": "7"})

                color_url = self.task.upload_fileobj(task_id, color_byte_arr, f"original/{folder}")
                depth_url = self.task.upload_fileobj(task_id, depth_byte_arr, f"original/{folder}")

                # 返回图像数据
                return JSONResponse(status_code=200,
                    content={"message": "Upload task image successfully",
                             "task_id": task_id,
                             "depth_url": depth_url,
                             "color_url": color_url,
                             "code": 0,
                             "intrinsic": self.upload_intrinsic(task_id)})
            except Exception as e:
                return JSONResponse(status_code=200,
                    content={"message": f"Didn't receive latest frame with error: {e}.", "code": "8"})

        @self.app.get("/hello")
        def hello():
            return JSONResponse(content={"msg": f"hello from {self.__class__.__name__}", })

        @self.app.get("/intrinsic")
        def hello():
            return JSONResponse(content=self.camera_param)

        class CalibrationAction(Enum):
            on = "on"
            off = "off"

        @self.app.get("/calibrate/{action}")
        def switch_calibration_mode(action: CalibrationAction):
            try:
                self.calibrationMode(action == CalibrationAction.on)
                return JSONResponse(content={"msg": f"Change calibration mode to {action.value}.", })
            except Exception as e:
                return JSONResponse(
                    content={"msg": f"Failed to change calibration mode to {action.value} with error {e}.", })

    def main_loop(self):
        while True:
            try:
                self.capture()
            except KeyboardInterrupt:
                continue
        self.stop_streaming()

    def run(self):
        try:
            self.initialize()

            if self.streaming:
                self.start_streaming()
                thread = threading.Thread(target=self.main_loop, daemon=True)
                thread.start()
        except Exception as e:
            logging.error(e)
            sys.exit(-1)

        return self.app
