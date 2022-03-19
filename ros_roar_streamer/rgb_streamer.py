#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

from typing import List, Optional, Tuple, List
import numpy as np

import sys, os
from pathlib import Path

sys.path.append(Path(os.getcwd()).parent.as_posix())
from .udp_receiver import UDPStreamer
from . import config as cfg
import struct
import time

bridge = CvBridge()
ip = cfg.config["ip_address"]
PUB_RATE = cfg.config["query_rate"]


class RGBCamStreamer(UDPStreamer):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.curr_image: Optional[np.ndarray] = None
        self.intrinsics: Optional[np.ndarray] = None

    def run_in_series(self, **kwargs):
        try:
            data = self.recv()
            if data is None:
                return
            img_data = data[16:]
            intrinsics = data[:16]
            fx, fy, cx, cy = (
                struct.unpack("f", intrinsics[0:4])[0],
                struct.unpack("f", intrinsics[4:8])[0],
                struct.unpack("f", intrinsics[8:12])[0],
                struct.unpack("f", intrinsics[12:16])[0],
            )
            self.intrinsics = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

            img = np.frombuffer(img_data, dtype=np.uint8)
            img = cv2.imdecode(img, cv2.IMREAD_UNCHANGED)
            if img is not None:
                self.curr_image = img

        except OSError:
            self.should_continue_threaded = False

        except Exception as e:
            self.logger.error(e)


class RGBStreamer(Node):
    def __init__(self):
        super().__init__("rgb_streamer")
        self.declare_parameter("ios_ip_address", "127.0.0.1")
        self.ios_ip_address = (
            self.get_parameter("ios_ip_address").get_parameter_value().string_value
        )
        self.ir_image_server = RGBCamStreamer(
            ios_address=self.ios_ip_address,
            pc_port=8001,
            name="world_rgb_streamer",
            update_interval=0.025,
            threaded=True,
        )
        self.rgb_image_pub = self.create_publisher(Image, "rgb_image", 10)
        self.rgb_info_pub = self.create_publisher(CameraInfo, "camera_info", 10)

        timer_period = cfg.config["query_rate"]  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ir_image_server = self.ir_image_server
        ir_image_server.run_in_series()
        if ir_image_server.curr_image is not None:
            img = ir_image_server.curr_image
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
            img = cv2.resize(img, dsize=(144,256))
            rgb_info_msg = CameraInfo()
            rgb_info_msg.header.frame_id = "sensors_link"
            # rgb_info_msg.header.seq = np.random.randint(0,10000000)
            rgb_info_msg.height = 256
            rgb_info_msg.width = 144
            rgb_info_msg.distortion_model = "plumb_bob"
            fx = float(ir_image_server.intrinsics[0, 0])
            fy = float(ir_image_server.intrinsics[1, 1])
            cx = float(ir_image_server.intrinsics[0, 2])
            cy = float(ir_image_server.intrinsics[1, 2])
            # print(cx, cy)
            rgb_info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            rgb_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            rgb_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            rgb_info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

            rgb_img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            rgb_info_msg.header.stamp = rgb_img_msg.header.stamp = (
                self.get_clock().now().to_msg()
            )
            rgb_img_msg.header.frame_id = "sensors_link"

            self.rgb_image_pub.publish(rgb_img_msg)
            self.rgb_info_pub.publish(rgb_info_msg)


def main(args=None):
    rclpy.init(args=args)

    rgb_streamer = RGBStreamer()
    try:
        rclpy.spin(rgb_streamer)
    except KeyboardInterrupt:
        pass
    rgb_streamer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
