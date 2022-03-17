from sympy import im
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros


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
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        os.path.join(
                            get_package_share_directory("roar_transforms"),
                            "launch",
                        ),
                        "roar_tf.launch.py",
                    ),
                ),
            ),
            launch.actions.IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    os.path.join(
                        os.path.join(
                            get_package_share_directory("roar_bot_description"),
                            "launch",
                        ),
                        "robot_streamer.launch.py",
                    ),
                ),
                launch_arguments={"gui": "False"}.items(),
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
                package="ros_roar_streamer",
                namespace="pointcloud_publisher",
                executable="pointcloud_publisher",
                name="pointcloud_publisher",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(rviz_path)],
            ),
            launch_ros.actions.ComposableNodeContainer(
                name="container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    # Driver itself
                    launch_ros.descriptions.ComposableNode(
                        package="depth_image_proc",
                        plugin="depth_image_proc::PointCloudXyzrgbNode",
                        name="point_cloud_xyzrgb_node",
                        remappings=[
                            ("rgb/camera_info", "/depth_streamer/camera_info"),
                            ("rgb/image_rect_color", "/rgb_streamer/rgb_image"),
                            (
                                "depth_registered/image_rect",
                                "/depth_streamer/depth_image",
                            ),
                            ("points", "/pointcloud"),
                        ],
                    ),
                ],
                output="screen",
            ),
        ]
    )
