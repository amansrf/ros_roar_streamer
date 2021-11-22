#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

import numpy as np
import sys, os
from pathlib import Path
import time
import config as cfg

sys.path.append(Path(os.getcwd()).parent.as_posix())
from udp_receiver import UDPStreamer
from data_structures_models import Transform, Vector3D

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

        except Exception as e:
            self.logger.error(e)


def StateStreamer():
    streamer = VehicleStateStreamer(ios_address=ip,
                                    port=8003,
                                    name="VehicleStateStreamer",
                                    update_interval=0.025,
                                    threaded=True)

    imu_pub = rospy.Publisher('iPhone_imu', Imu, queue_size=10)
    odom_pub = rospy.Publisher('iPhone_odom', Odometry, queue_size=10)
    rospy.init_node('state_streamer', anonymous=False)
    
    rate = rospy.Rate(PUB_RATE) # in Hz

    while not rospy.is_shutdown():
        streamer.run_in_series()
        # print(streamer.transform, streamer.velocity)

        q = quaternion_from_euler(streamer.transform.rotation.roll, 
                              streamer.transform.rotation.pitch,
                              streamer.transform.rotation.yaw)

        ### Constructing Imu Message
        imu_msg = Imu()
        imu_msg.header.frame_id = 'base_link'
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.orientation = Quaternion(*q)
        imu_msg.orientation_covariance = [-1,0,0,0,0,0,0,0,0]
        imu_msg.angular_velocity.x = streamer.gyro.x
        imu_msg.angular_velocity.y = streamer.gyro.y
        imu_msg.angular_velocity.z = streamer.gyro.z
        imu_msg.linear_acceleration.x = G*streamer.acceleration.x
        imu_msg.linear_acceleration.y = G*streamer.acceleration.y
        imu_msg.linear_acceleration.z = G*streamer.acceleration.z
        ### Constructing Odom Message
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.pose.pose.position.x = streamer.transform.location.x
        odom_msg.pose.pose.position.y = streamer.transform.location.y
        odom_msg.pose.pose.position.z = streamer.transform.location.z
        odom_msg.pose.pose.orientation = Quaternion(*q)
        odom_msg.twist.twist.linear.x = streamer.velocity.x
        odom_msg.twist.twist.linear.y = streamer.velocity.y
        odom_msg.twist.twist.linear.z = streamer.velocity.z
        odom_msg.twist.twist.angular.x = streamer.gyro.x
        odom_msg.twist.twist.angular.y = streamer.gyro.y
        odom_msg.twist.twist.angular.z = streamer.gyro.z

        imu_pub.publish(imu_msg)
        odom_pub.publish(odom_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        StateStreamer()
    except rospy.ROSInterruptException:
        pass

