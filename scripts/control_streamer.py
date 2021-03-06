#!/usr/bin/env python3

import rospy
from carla_msgs.msg import CarlaEgoVehicleControl
import socket
import config as cfg
import numpy as np


udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
host = cfg.config["ip_address"] ### Local address of iPhone
port = 8004 ### Port for Control Signals on iPhone

def send_control_over_udp(data):
    #Clipping Steering and throttle values to safe region.
    data.steer = np.clip(data.steer, -1, 1)
    data.throttle = np.clip(data.throttle, -1, 1)

    #Creating Message Packet and Sending
    msg = str(data.throttle) + "," + str(data.steer)
    udp_socket.sendto(msg.encode('utf-8'), (host,port))

def ControlStreamer():
    rospy.init_node('control_streamer', anonymous=False)
    rospy.Subscriber('/carla/ego_vehicle/vehicle_control_cmd_manual', CarlaEgoVehicleControl, send_control_over_udp)
    rospy.spin()

if __name__ == '__main__':
    try:
        ControlStreamer()
    except rospy.ROSInterruptException:
        pass
