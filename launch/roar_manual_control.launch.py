from sympy import im
from launch import LaunchDescription
from launch_ros.actions import Node
import os 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    base_path = os.path.realpath(get_package_share_directory('ros_roar_streamer')) # also tried without realpath
    rviz_path=base_path+'/default_config.rviz'
    print(rviz_path)
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
            ),
            Node(
                package='rviz2', 
                executable='rviz2', 
                name="rviz2", 
                arguments=['-d', str(rviz_path)]),
            Node(
                package="ros_roar_streamer",
                namespace="manual_drive_with_pygame",
                executable="manual_drive_with_pygame",
                name="manual_drive_with_pygame"
            )
        ]
    )