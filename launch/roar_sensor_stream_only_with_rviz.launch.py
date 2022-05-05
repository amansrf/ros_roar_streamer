from sympy import im
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    base_path = os.path.realpath(
        get_package_share_directory("ros_roar_streamer")
    )  # also tried without realpath
    rviz_path = base_path + "/config/default_config.rviz"
    namespace = launch.substitutions.LaunchConfiguration('namespace')
    map_yaml_file = launch.substitutions.LaunchConfiguration('map')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    autostart = launch.substitutions.LaunchConfiguration('autostart')
    params_file = launch.substitutions.LaunchConfiguration('params_file')
    lifecycle_nodes = ['map_server']
    params_file = launch.substitutions.LaunchConfiguration('params_file')

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
        }
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                'namespace', default_value='',
                description='Top-level namespace'),

            launch.actions.DeclareLaunchArgument(
                'map',
                default_value=os.path.join(base_path, 'maps', 'turtlebot3_world.yaml'),
                description='Full path to map yaml file to load'),

            launch.actions.DeclareLaunchArgument(
                'use_sim_time', default_value='false',
                description='Use simulation (Gazebo) clock if true'),

            launch.actions.DeclareLaunchArgument(
                'autostart', default_value='true',
                description='Automatically startup the nav2 stack'),
            
            launch.actions.DeclareLaunchArgument(
                'params_file',
                default_value='/home/roar/roar/roar_ros/src/ros_roar_streamer/params/nav2_params.yaml',
                description='Full path to the ROS2 parameters file to use'
            ),
            launch.actions.DeclareLaunchArgument(
                name="ios_ip_address", default_value="127.0.0.1",
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
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
            ),
            # Node(
            #     package="nav2_costmap_2d",
            #     name="costmap_node",   
            #     parameters=[
            #         os.path.join(base_path, 'params', 'nav2_params.yaml')
            #     ], 
            # ),
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                parameters=[configured_params],
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]
            ),
            # Node(
            #     package="ros_roar_streamer",
            #     namespace="pointcloud_publisher",
            #     executable="pointcloud_publisher",
            #     name="pointcloud_publisher",
            # ),
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
