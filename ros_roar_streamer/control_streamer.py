#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from carla_msgs.msg import CarlaEgoVehicleControl

import socket

from . import config as cfg
import numpy as np

# Defining required constants
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
host = cfg.config["ip_address"] ### Local address of iPhone
port = 8004 ### Port for Control Signals on iPhone

class ControlStreamer(Node):

    def __init__(self):
        super().__init__('control_streamer')
        self.subscription = self.create_subscription(
            CarlaEgoVehicleControl,
            '/carla/ego_vehicle/vehicle_control_cmd_manual',
            self.send_control_over_udp,
            1)
        self.subscription  # prevent unused variable warning
    
    def send_control_over_udp(self, data):
        # Debug Only
        self.get_logger().info('I heard: "%s"' % data)

        #Clipping Steering and throttle values to safe region.
        data.steer = np.clip(data.steer, -1, 1)
        data.throttle = np.clip(data.throttle, -1, 1)

        #Creating Message Packet and Sending
        msg = str(data.throttle) + "," + str(data.steer)
        udp_socket.sendto(msg.encode('utf-8'), (host,port))

def main(args=None):
    rclpy.init(args=args)

    control_streamer = ControlStreamer()

    rclpy.spin(control_streamer)

    control_streamer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

