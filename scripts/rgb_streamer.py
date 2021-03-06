#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge

from typing import List, Optional, Tuple, List
import numpy as np
import cv2
import sys, os
from pathlib import Path

sys.path.append(Path(os.getcwd()).parent.as_posix())
from udp_receiver import UDPStreamer
import config as cfg
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
            fx, fy, cx, cy = struct.unpack('f', intrinsics[0:4])[0], \
                             struct.unpack('f', intrinsics[4:8])[0], \
                             struct.unpack('f', intrinsics[8:12])[0], \
                             struct.unpack('f', intrinsics[12:16])[0]
            self.intrinsics = np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ])

            img = np.frombuffer(img_data, dtype=np.uint8)
            img = cv2.imdecode(img, cv2.IMREAD_UNCHANGED)
            if img is not None:
                self.curr_image = img

        except OSError:
            self.should_continue_threaded = False

        except Exception as e:
            self.logger.error(e)


def RGBStreamer():
    ir_image_server = RGBCamStreamer(ios_address=ip,
                                     pc_port=8001,
                                     name="world_rgb_streamer",
                                     update_interval=0.025,
                                     threaded=True)

    RGB_image_pub = rospy.Publisher('rgb_image', Image, queue_size=10)
    rgb_info_pub = rospy.Publisher('rgb_img_info', CameraInfo, queue_size=10)
    rospy.init_node('rgb_streamer', anonymous=False)

    rate = rospy.Rate(PUB_RATE) # in Hz

    while not rospy.is_shutdown():
        ir_image_server.run_in_series()
        if ir_image_server.curr_image is not None:
            img = ir_image_server.curr_image
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
            cv2.imshow("img", img)
            cv2.waitKey(1)

            rgb_info_msg = CameraInfo()
            rgb_info_msg.header.frame_id = 'base_link'
            # rgb_info_msg.header.seq = np.random.randint(0,10000000)
            rgb_info_msg.height = 256
            rgb_info_msg.width = 144
            rgb_info_msg.distortion_model = 'plumb_bob'
            fx = ir_image_server.intrinsics[0,0]
            fy = ir_image_server.intrinsics[1,1]
            cx = ir_image_server.intrinsics[0,2]
            cy = ir_image_server.intrinsics[1,2]
            print(cx, cy)
            rgb_info_msg.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            rgb_info_msg.D = [0, 0, 0, 0, 0]
            rgb_info_msg.R = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]
            rgb_info_msg.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1.0, 0]

            rgb_img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            rgb_img_msg.header.stamp = rospy.Time.now()
            RGB_image_pub.publish(rgb_img_msg)
            rgb_info_pub.publish(rgb_info_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        RGBStreamer()
    except rospy.ROSInterruptException:
        pass
