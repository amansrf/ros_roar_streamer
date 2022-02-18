#!/usr/bin/env python3


from sympy import im
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

from std_msgs.msg import String

import numpy as np
import sys, os
from pathlib import Path
import time
from . import config as cfg

sys.path.append(Path(os.getcwd()).parent.as_posix())
from . udp_receiver import UDPStreamer
from . data_structures_models import Transform, Vector3D

G = cfg.config["G"]
ip = cfg.config["ip_address"]
PUB_RATE = cfg.config["query_rate"]

class VehicleStateStreamer(UDPStreamer):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.transform = Transform()
        self.velocity = Vector3D()
        self.acceleration = Vector3D()
        self.gyro = Vector3D()
        
        self.ix = float(0)
        self.iy = float(0)
        self.iz = float(0)
        self.r = float(0)

    def run_in_series(self, **kwargs):
        try:
            data = self.recv()
            if data is None:
                return
            d = [float(s) for s in data.decode('utf-8').split(",")]
            # d = np.frombuffer(data, dtype=np.float32)
            self.transform.location.x = d[0]
            self.transform.location.y = d[1]
            self.transform.location.z = d[2]
            self.transform.rotation.roll = d[3]
            self.transform.rotation.pitch = d[4]
            self.transform.rotation.yaw = d[5]
            self.velocity.x = d[6]
            self.velocity.y = d[7]
            self.velocity.z = d[8]
            self.acceleration.x = d[9]
            self.acceleration.y = d[10]
            self.acceleration.z = d[11]
            self.gyro.x = d[12]
            self.gyro.y = d[13]
            self.gyro.z = d[14]

            self.hall_effect_velocity = d[15]

            self.recv_time = d[16]
            self.ix = d[17]
            self.iy = d[18]
            self.iz = d[19]
            self.r = d[20]

        except Exception as e:
            self.logger.error(e)

class StateStreamer(Node):

    def __init__(self):
        super().__init__('state_streamer')
        self.streamer = VehicleStateStreamer(ios_address=ip,
                                    port=8003,
                                    name="VehicleStateStreamer",
                                    update_interval=0.025,
                                    threaded=True)
        self.imu_pub = self.create_publisher(Imu, '/demo/imu', 10)
        self.odom_pub = self.create_publisher(Odometry, '/demo/odom', 10)
        self.transform_br = TransformBroadcaster(self) 

        timer_period = cfg.config["query_rate"]  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        streamer = self.streamer
        streamer.run_in_series()

        ### DEBUG ONLY
        # q = quaternion_from_euler(streamer.transform.rotation.roll, 
        #                       streamer.transform.rotation.pitch,
        #                       streamer.transform.rotation.yaw)
        
        ### Constructing Imu Message
        # imu_msg = Imu()
        # imu_msg.header.frame_id = 'base_link'
        # imu_msg.header.stamp = self.get_clock().now().to_msg()
        # # imu_msg.orientation = Quaternion(q[0], q[1], q[2], q[3])
        # imu_msg.orientation = Quaternion()
        # imu_msg.orientation.x = streamer.ix
        # imu_msg.orientation.y = streamer.iy
        # imu_msg.orientation.z = streamer.iz
        # imu_msg.orientation.w = streamer.r
        # imu_msg.orientation_covariance = [-1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        # imu_msg.angular_velocity.x = float(streamer.gyro.x)
        # imu_msg.angular_velocity.y = float(streamer.gyro.y)
        # imu_msg.angular_velocity.z = float(streamer.gyro.z)
        # imu_msg.linear_acceleration.x = G*streamer.acceleration.x
        # imu_msg.linear_acceleration.y = G*streamer.acceleration.y
        # imu_msg.linear_acceleration.z = G*streamer.acceleration.z
        
        
        # ### Constructing Odom Message
        # odom_msg = Odometry()
        # odom_msg.header.frame_id = 'world'
        # odom_msg.child_frame_id = 'base_link'
        # odom_msg.header.stamp = self.get_clock().now().to_msg()
        # odom_msg.pose.pose.position.x = streamer.transform.location.x
        # odom_msg.pose.pose.position.y = streamer.transform.location.y
        # odom_msg.pose.pose.position.z = streamer.transform.location.z
        # odom_msg.pose.pose.orientation = Quaternion()
        # odom_msg.pose.pose.orientation.x = streamer.ix
        # odom_msg.pose.pose.orientation.y = streamer.iy
        # odom_msg.pose.pose.orientation.z = streamer.iz
        # odom_msg.pose.pose.orientation.w = streamer.r
        # odom_msg.twist.twist.linear.x = float(streamer.velocity.x)
        # odom_msg.twist.twist.linear.y = float(streamer.velocity.y)
        # odom_msg.twist.twist.linear.z = float(streamer.velocity.z)
        # odom_msg.twist.twist.angular.x = float(streamer.gyro.x)
        # odom_msg.twist.twist.angular.y = float(streamer.gyro.y)
        # odom_msg.twist.twist.angular.z = float(streamer.gyro.z)

        # ### construct transform information
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = streamer.transform.location.x
        t.transform.translation.y = streamer.transform.location.y
        t.transform.translation.z = streamer.transform.location.z

        t.transform.rotation.x = streamer.ix
        t.transform.rotation.y = streamer.iy
        t.transform.rotation.z = streamer.iz
        t.transform.rotation.w = streamer.r

        self.transform_br.sendTransform(t)
        # self.imu_pub.publish(imu_msg)
        # self.odom_pub.publish(odom_msg)

        # self.get_logger().info('Publishing odom: "%s"' % odom_msg)
        # self.get_logger().info('Publishing imu: "%s"' % imu_msg)

def main(args=None):
    rclpy.init(args=args)

    state_streamer = StateStreamer()
    try:
        rclpy.spin(state_streamer)
    except KeyboardInterrupt:
        pass 


    state_streamer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()