from sympy import im
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.descriptions import ParameterValue
import launch_ros
import os
from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    base_path = os.path.realpath(get_package_share_directory('ros_roar_streamer')) # also tried without realpath
    rviz_path=base_path+'/config/all.rviz'
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ros_roar_streamer').find('ros_roar_streamer')
    default_rviz_config_path = rviz_path

    default_model_path = (Path(pkg_share) / "URDF" / "roar_bot.urdf.xacro").as_posix()

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
                'robot_description': ParameterValue(
            Command(['xacro ', str(default_model_path)]), value_type=str
        )
        }]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription(
        [
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),

        joint_state_publisher_node,
        robot_state_publisher_node,
        robot_localization_node,
        rviz_node,
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
            Node(package="ros_roar_streamer",
            namespace="pointcloud_publisher",
            executable="pointcloud_publisher",
            name="pointcloud_publisher"
        )
        ]
    )