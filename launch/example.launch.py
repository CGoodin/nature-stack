import os

import launch.conditions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_launch_rviz = LaunchConfiguration("auto_launch_rviz")
    display_type = LaunchConfiguration('display_type')
    urdf = os.path.join(get_package_share_directory('avt_341'), 'config', 'avt_bot.urdf')
    rviz_config_path = os.path.join(get_package_share_directory('avt_341'), 'rviz', 'avt_341.rviz')

    waypoints_config = os.path.join(
        get_package_share_directory('avt_341'),
        'config',
        'waypoints.yaml'
    )

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    launch_description = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('auto_launch_rviz', default_value='True'),
        DeclareLaunchArgument('display_type', default_value='rviz'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        Node(
            package='avt_341',
            executable='avt_bot_state_publisher_node',
            name='state_publisher'),
        Node(
            package='avt_341',
            executable='avt_341_perception_node',
            name='perception_node',
            output='screen',
            parameters=[{
                'use_elevation': False,
                'slope_threshold': 0.5,
                'grid_height': 200.0,
                'grid_width': 200.0,
                'grid_llx': -100.0,
                'grid_lly': -100.0,
                'grid_res': 0.5,
                'overhead_clearance': 7.0,
                'warmup_time': 5.0,
                'use_registered': True
            }],
        ),
        Node(
            package='avt_341',
            executable='avt_341_control_node',
            name='vehicle_control_node',
            output='screen',
            parameters=[{
                'vehicle_wheelbase': 2.72,
                'vehicle_max_steer_angle_degrees': 30.0,
                'steering_coefficient': 4.5,
                'vehicle_speed': 5.0,
            }],
        ),
        Node(
            package='avt_341',
            executable='avt_341_global_path_node',
            name='avt_341_global_path_node',
            output='screen',
            parameters=[{
                'goal_dist': 5.0,
                'global_lookahead': 75.0,
                'shutdown_behavior': 2,
                'display': display_type
            }, waypoints_config],
        ),
        Node(
            package='avt_341',
            executable='avt_341_local_planner_node',
            name='local_planner_node',
            output='screen',
            parameters=[{
                'path_look_ahead': 30.0,
                'vehicle_width': 3.0,
                'num_paths': 21,
                'max_steer_angle': 0.5,
                'output_path_step': 0.5,
                'path_integration_step': 0.35,
                'dilation_factor': 1,
                'w_c': 0.0,
                'w_s': 0.4,
                'w_d': 0.0,
                'w_r': 0.2,
                'rate': 50.0,
                'trim_path': True,
                'display': display_type,
            }],
        ),
        Node(
            package='avt_341',
            executable='avt_341_sim_test_node',
            name='sim_test_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     condition=launch.conditions.IfCondition(auto_launch_rviz),
        #     arguments=["-d", rviz_config_path])
    ])

    return launch_description
