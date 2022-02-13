#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

from typing import List, Optional, Tuple, List
import numpy as np
import cv2
import sys, os
from pathlib import Path

sys.path.append(Path(os.getcwd()).parent.as_posix())
from . udp_receiver import UDPStreamer
from . import config as cfg
import struct
import time

bridge = CvBridge()
ip = cfg.config["ip_address"]
PUB_RATE = cfg.config["query_rate"]

class DepthCamStreamer(UDPStreamer):
    def __init__(self, resize: Optional[Tuple] = None, **kwargs):
        super().__init__(**kwargs)
        self.curr_image: Optional[np.ndarray] = None
        self.resize = resize
        self.intrinsics: Optional[np.ndarray] = None

    def run_in_series(self, **kwargs):
        try:
            data = self.recv()
            if data is None:
                return
            img_data = data[16:]
            intrinsics = data[0:16]
            fx, fy, cy, cx = struct.unpack('f', intrinsics[0:4])[0], \
                             struct.unpack('f', intrinsics[4:8])[0], \
                             struct.unpack('f', intrinsics[8:12])[0], \
                             struct.unpack('f', intrinsics[12:16])[0]
            self.intrinsics = np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ])
            img = np.frombuffer(img_data, dtype=np.float32)
            if img is not None:
                self.curr_image = np.rot90(img.reshape((144, 256)), k=-1)

        except OSError:
            self.should_continue_threaded = False
        except Exception as e:
            self.logger.error(e)


class DepthStreamer(Node):

    def __init__(self):
        super().__init__('depth_streamer')
        self.ir_image_server = DepthCamStreamer(ios_address=ip,
                                       port=8002,
                                       name="world_depth_streamer",
                                       update_interval=0.05,
                                       threaded=True)
        self.depth_image_pub = self.create_publisher(Image, 'depth_image', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, 'depth_img_info', 10)

        timer_period = cfg.config["query_rate"]  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ir_image_server = self.ir_image_server
        ir_image_server.run_in_series()
        if ir_image_server.curr_image is not None:
            img = ir_image_server.curr_image
            cv2.imshow("img", img)
            cv2.waitKey(1)

            depth_info_msg = CameraInfo()
            depth_info_msg.header.frame_id = 'base_link'
            # depth_info_msg.header.seq = np.random.randint(0,10000000)
            depth_info_msg.height = 256
            depth_info_msg.width = 144
            depth_info_msg.distortion_model = 'plumb_bob'
            fx = float(ir_image_server.intrinsics[0,0])
            fy = float(ir_image_server.intrinsics[1,1])
            cx = float(ir_image_server.intrinsics[0,2])
            cy = float(ir_image_server.intrinsics[1,2])
            # print(cx, cy)
            depth_info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            depth_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            depth_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            depth_info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

            depth_img_msg = bridge.cv2_to_imgmsg(img, encoding="passthrough")
            depth_img_msg.header.frame_id ='base_link'
            # Setting time of both info and image to get same timestamp on both
            depth_img_msg.header.stamp = depth_info_msg.header.stamp = self.get_clock().now().to_msg()

            self.depth_image_pub.publish(depth_img_msg)
            self.depth_info_pub.publish(depth_info_msg)

def main(args=None):
    rclpy.init(args=args)

    depth_streamer = DepthStreamer()

    rclpy.spin(depth_streamer)

    depth_streamer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()