from sympy import im
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="ros_roar_streamer",
                namespace="depth_streamer",
                executable="depth_streamer",
                name="depth_streamer"
            ),
            Node(package="ros_roar_streamer",
                namespace="rgb_streamer",
                executable="rgb_streamer",
                name="rgb_streamer"
            ),
             Node(package="ros_roar_streamer",
                namespace="control_streamer",
                executable="control_streamer",
                name="control_streamer"
            ),
             Node(package="ros_roar_streamer",
                namespace="state_streamer",
                executable="state_streamer",
                name="state_streamer"
            )
        ]
    )