from sympy import im
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import launch


def generate_launch_description():

    base_path = os.path.realpath(
        get_package_share_directory("ros_roar_streamer")
    )  # also tried without realpath
    rviz_path = base_path + "/config/default_config.rviz"

    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                name="ios_ip_address", default_value="127.0.0.1"
            ),
            launch.actions.DeclareLaunchArgument(
                name="max_reverse_throttle", default_value="-1.0"
            ),
            launch.actions.DeclareLaunchArgument(
                name="max_forward_throttle", default_value="1.0"
            ),
            launch.actions.DeclareLaunchArgument(
                name="max_steering", default_value="1.0"
            ),
            launch.actions.DeclareLaunchArgument(
                name="steering_offset", default_value="0.0"
            ),
            Node(
                package="ros_roar_streamer",
                namespace="depth_streamer",
                executable="depth_streamer",
                name="depth_streamer",
                parameters=[
                    {
                        "ios_ip_address": launch.substitutions.LaunchConfiguration(
                            "ios_ip_address"
                        )
                    },
                ],
            ),
            Node(
                package="ros_roar_streamer",
                namespace="rgb_streamer",
                executable="rgb_streamer",
                name="rgb_streamer",
                parameters=[
                    {
                        "ios_ip_address": launch.substitutions.LaunchConfiguration(
                            "ios_ip_address"
                        )
                    },
                ],
            ),
            Node(
                package="ros_roar_streamer",
                namespace="control_streamer",
                executable="control_streamer",
                name="control_streamer",
                parameters=[
                    {
                        "ios_ip_address": launch.substitutions.LaunchConfiguration(
                            "ios_ip_address"
                        )
                    },
                ],
            ),
            Node(
                package="ros_roar_streamer",
                namespace="state_streamer",
                executable="state_streamer",
                name="state_streamer",
                parameters=[
                    {
                        "ios_ip_address": launch.substitutions.LaunchConfiguration(
                            "ios_ip_address"
                        )
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(rviz_path)],
            ),
            Node(
                package="ros_roar_streamer",
                namespace="manual_drive_with_pygame",
                executable="manual_drive_with_pygame",
                name="manual_drive_with_pygame",
                parameters=[
                    {
                        "max_reverse_throttle": launch.substitutions.LaunchConfiguration(
                            "max_reverse_throttle"
                        ),
                        "max_forward_throttle": launch.substitutions.LaunchConfiguration(
                            "max_forward_throttle"
                        ),
                        "max_steering": launch.substitutions.LaunchConfiguration(
                            "max_steering"
                        ),
                        "steering_offset": launch.substitutions.LaunchConfiguration(
                            "steering_offset"
                        ),
                    }
                ],
            ),
        ]
    )
