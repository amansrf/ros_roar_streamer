#!/usr/bin/env python3

from sympy import im
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import message_filters
import numpy as np
import open3d as o3d 
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import struct 
import ctypes
from ctypes import * # convert float to uint32
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
import cv2

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__("pointcloud_publisher")
        self.depth_image_sub = message_filters.Subscriber(self, Image, "/depth_streamer/depth_image")
        self.depth_image_info_sub = message_filters.Subscriber(self,CameraInfo,"/depth_streamer/camera_info")
        self.rgb_image_sub = message_filters.Subscriber(self, Image, "/rgb_streamer/rgb_image")
        self.odom = message_filters.Subscriber(self, Odometry, "/odom")
        self.bridge = CvBridge()
        self.ats = message_filters.ApproximateTimeSynchronizer(
            [self.depth_image_sub, self.depth_image_info_sub, self.rgb_image_sub, self.odom],
            queue_size=5, slop=0.1)
        self.ats.registerCallback(self.on_frames_received)

        self.point_cloud_pub = self.create_publisher(PointCloud2, '/pointcloud', 10)
        
    def on_frames_received(self, 
                           depth_image_msg:Image, 
                           depth_image_info_msg:CameraInfo, 
                           rgb_image_msg:Image,
                           odom: Odometry):
        depth_img = self.bridge.imgmsg_to_cv2(depth_image_msg, 
                    desired_encoding="passthrough").astype(np.float32)
        
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_image_msg, desired_encoding="bgr8")
        rgb_img = cv2.resize(rgb_img, dsize=(depth_img.shape[1], depth_img.shape[0]))
        depth_intrinsics = np.reshape(depth_image_info_msg.k, (3,3))
        rgb = o3d.geometry.Image(rgb_img)
        depth = o3d.geometry.Image(depth_img)

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color=rgb,
                                                                  depth=depth,
                                                                  convert_rgb_to_intensity=False,
                                                                  depth_scale=1,
                                                                  depth_trunc=100)
        
        intrinsic = o3d.camera.PinholeCameraIntrinsic(width=rgb_img.shape[0],
                                                      height=rgb_img.shape[1],
                                                      fx=depth_intrinsics[0][0],  # added this hack to flip it
                                                      fy=depth_intrinsics[1][1],  # added this hack to flip it
                                                      cx=depth_intrinsics[0][2],
                                                      cy=depth_intrinsics[1][2])
        extrinsics = np.eye(4)
        quat = np.array([odom.pose.pose.orientation.x,
                         odom.pose.pose.orientation.y,
                         odom.pose.pose.orientation.z,
                         odom.pose.pose.orientation.w])
        trans = np.array([odom.pose.pose.position.x,
                          odom.pose.pose.position.y,
                          odom.pose.pose.position.z,
                          1
        ])
        rotation_matrix = o3d.geometry.get_rotation_matrix_from_quaternion(quat)
        extrinsics[0:3,0:3] = rotation_matrix
        extrinsics[3, :] = trans
        pcd: o3d.geometry.PointCloud = o3d.geometry.PointCloud. \
            create_from_rgbd_image(image=rgbd,
                                   intrinsic=intrinsic,
                                   extrinsic=extrinsics)

        points = np.asarray(pcd.points).astype(np.float32)
        # colors = np.asarray(pcd.points)
        # colors = np.floor(colors*255).astype(np.uint8) # nx3 matrix
        # colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2]  
        pc2 = point_cloud(points, parent_frame="base_link", stamp=self.get_clock().now().to_msg())
        self.point_cloud_pub.publish(pc2)

def point_cloud(points, parent_frame, stamp):
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes() 

    # The fields specify what the bytes represents. The first 4 bytes 
    # represents the x-coordinate, the next 4 the y-coordinate, etc.
    fields = [PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    # The PointCloud2 message also has a header which specifies which 
    # coordinate frame it is represented in. 
    header = Header(frame_id=parent_frame)
    header.stamp = stamp
    return PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]), 
        data=data
    )


def main(args = None):
    rclpy.init(args=args)
    operator = PointCloudPublisher()
    try:
        rclpy.spin(operator)
    except KeyboardInterrupt:
        pass 
    operator.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
