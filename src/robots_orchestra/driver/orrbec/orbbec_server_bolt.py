import json
import logging
import threading
import time
from typing import Optional
import base64
import numpy as np
from fastapi import FastAPI, HTTPException, Request, Response
from fastapi.responses import JSONResponse, FileResponse, HTMLResponse
from pydantic import BaseModel
import pyorbbecsdk
from pyorbbecsdk import *
import uvicorn
from PIL import Image
import io
from starlette.responses import StreamingResponse
import cv2

from Chin.core.HttpServerUtil import DummyServer
from Chin.core.TaskUtil import TaskUtil

from robots_orchestra.driver.orrbec.camera_server_base import CameraServer

def ob_camera_param_to_dict(param: OBCameraParam):
    ret = [
        # {'d2c_extrinsic': self.d2c_extrinsic,},
        {
            "type": "color",
            "K": get_intrinsic_matrix(param.rgb_intrinsic),
            "d": get_distortion(param.rgb_distortion),
            "width": param.rgb_intrinsic.width,
            "height": param.rgb_intrinsic.height,
            "transform": np.eye(4).tolist(),
        },
        {

            "type": "depth",
            "K": get_intrinsic_matrix(param.depth_intrinsic),
            "d": get_distortion(param.depth_distortion),
            "width": param.depth_intrinsic.width,
            "height": param.depth_intrinsic.height,
            "transform": get_transform_matrix(param.transform),
            "scale": 1.0,
        },
        
    ]

    return ret


def get_intrinsic_matrix(ob_intrinsic):
    return [[ob_intrinsic.fx, 0, ob_intrinsic.cx], [0, ob_intrinsic.fy, ob_intrinsic.cy], [0, 0, 1]]


def get_distortion(ob_distortion):
    return [ob_distortion.k1, ob_distortion.k2, ob_distortion.k3, ob_distortion.k4,
            ob_distortion.k5, ob_distortion.k6, ob_distortion.p1, ob_distortion.p2]


def get_transform_matrix(ob_transform):
    ret = np.eye(4)

    ret[:3, :3] = ob_transform.rot
    ret[:3, 3] = ob_transform.transform

    return ret.tolist()


class OrbbecGrabber(CameraServer):
    def __init__(self):
        self.camera_param = None
        self.server = DummyServer()
        self.server.register_service("OrbbecGrabber", "A service to grab orrbec images using pyorbbecsdk.", "1.0.0", {
            "color": {"method": "POST,GET", "description": "upload a latest color image to storage into given task id"},
            "depth": {"method": "POST,GET", "description": "upload a latest depth image to storage into given task id"},
            "image": {"method": "POST,GET",
                      "description": "upload a latest depth and color image to storage into given task id"},
            "intrinsic": {"method": "GET", "description": "Get intrinsic parameter of camera."},
        })

        self.task = TaskUtil()

        self.device: Optional[Device] = None
        self.pipeline: Optional[Pipeline] = None
        self.device_lock = threading.Lock()

        self.app = self.server.create_app("OrbbecGrabber", "1.0.0")

        super().__init__(self.app, self.task, True, frame_rate=30, resolution=(1280, 720))

    def initialize(self):
        ctx = Context()
        ctx.set_logger_level(OBLogLevel.ERROR)
        ctx.set_device_changed_callback(self.on_device_changed_callback)
        device_list = ctx.query_devices()
        self.on_device_connected_callback(device_list)

    def start_streaming(self):
        config = Config()
        if self.device is None:
            logging.error("No device connected")
            return
        self.pipeline = Pipeline(self.device)
        logging.info("try to enable stream")
        try:
            profile_list = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            try:
                color_profile: VideoStreamProfile = profile_list.get_video_stream_profile(self.width, self.height,
                    OBFormat.RGB, self.frame_rate)
            except OBError as e:
                logging.error(e)
                color_profile = profile_list.get_default_video_stream_profile()
            config.enable_stream(color_profile)
        except Exception as e:
            logging.error(e)
        try:
            profile_list = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            try:
                depth_profile: VideoStreamProfile = profile_list.get_video_stream_profile(self.width, self.height,
                    OBFormat.Y16, self.frame_rate)
            except OBError as e:
                logging.error(e)
                depth_profile = profile_list.get_default_video_stream_profile()
            config.enable_stream(depth_profile)
        except Exception as e:
            logging.error(e)
        try:
            config.set_align_mode(OBAlignMode.SW_MODE)
            self.pipeline.enable_frame_sync()

            
            self.align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)

        except Exception as e:
            logging.error(e)
        logging.info("try to start stream")
        self.pipeline.start(config)

        self.color_profile = color_profile
        self.depth_profile = depth_profile

        self.d2c_extrinsic = get_transform_matrix(depth_profile.get_extrinsic_to(color_profile))
        camera_param = self.pipeline.get_camera_param()
        self.camera_param = ob_camera_param_to_dict(camera_param)
        # self.camera_param = [{'d2c_extrinsic': self.d2c_extrinsic,}] + self.camera_param
        # np.savetxt(save_path+'d2c_extrinsic.txt', self.d2c_extrinsic)

    def stop_streaming(self):
        if self.pipeline is None:
            logging.error("Pipeline is not started")
            return
        self.pipeline.stop()
        self.pipeline = None

    def on_device_connected_callback(self, device_list: DeviceList):
        if device_list.get_count() == 0:
            return
        print("Device connected")
        with self.device_lock:
            if self.device is not None:
                logging.error("Device is already connected")
                return
            logging.info("Try to get device")
            self.device = device_list.get_device_by_index(0)
            self.start_streaming()

    def on_device_disconnected_callback(self, device_list: DeviceList):
        if device_list.get_count() == 0:
            return
        print("Device disconnected")
        with self.device_lock:
            self.device = None
            self.pipeline = None

    def on_new_frame_callback(self, frame: Frame):
        if frame is None:
            return
        if frame.get_type() == OBFrameType.COLOR_FRAME:
            self.latest_color_frame = np.frombuffer(frame.get_data(), np.uint8).reshape(
                (frame.get_height(), frame.get_width(), 3))
            self.latest_color_recv_time = time.time()
            
        elif frame.get_type() == OBFrameType.DEPTH_FRAME:
            self.latest_depth_frame = np.frombuffer(frame.get_data(), np.uint16).reshape(
                (frame.get_height(), frame.get_width()))
            self.latest_depth_recv_time = time.time()
            

    def on_device_changed_callback(self, disconn_device_list: DeviceList, conn_device_list: DeviceList):
        self.on_device_connected_callback(conn_device_list)
        self.on_device_disconnected_callback(disconn_device_list)

    def generate_video_feed(self, depth=False):
        while True:

            time.sleep(1 / self.frame_rate)
            if not self.pipeline:
                break

            # 编码为JPEG
            img_bytes = self.get_image(depth=depth)

            # 构造 multipart 响应
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + img_bytes.getvalue() + b"\r\n")

    def capture(self):
        while True:
            with self.device_lock:
                if self.pipeline is not None and self.device is not None:
                    frames: FrameSet = self.pipeline.wait_for_frames(100)
                else:
                    return
            if frames is None:
                time.sleep(0.001)
                return
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            frames = self.align_filter.process(frames)
            if not frames:
                continue
            frames  = frames.as_frame_set()

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            break


        self.on_new_frame_callback(color_frame)
        self.on_new_frame_callback(depth_frame)
        # try:
        #     color_data = cv2.cvtColor( self.latest_color_frame, cv2.COLOR_BGR2RGB)
        #     cv2.imwrite(save_path+'current_color.png', color_data)
        #     depth_data = self.latest_depth_frame.astype(np.float32) 
        #     np.save(save_path+'current_depth.npy', depth_data)
        # except:
        #     pass


        # self.d2c_extrinsic = get_transform_matrix(self.depth_profile.get_extrinsic_to(self.color_profile))
        # camera_param = self.pipeline.get_camera_param()
        # self.camera_param = ob_camera_param_to_dict(camera_param)
        # # self.camera_param = [{'d2c_extrinsic': self.d2c_extrinsic,}] + self.camera_param
        # np.savetxt(save_path+'d2c_extrinsic.txt', self.d2c_extrinsic)


# grabber = OrbbecGrabber()

# app = grabber.run()

# if __name__ == "__main__":
#     uvicorn.run(app, host="0.0.0.0", port=8005)
