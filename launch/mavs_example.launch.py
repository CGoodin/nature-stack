import os

import launch
import launch_ros.actions
import launch.conditions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch.conditions
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():

    nature_package_dir = get_package_share_directory('nature')
    urdf = os.path.join(nature_package_dir, 'config', 'example_bot.urdf')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    waypoints_file = os.path.join(
        nature_package_dir,
        'config',
        'waypoints.yaml'
    )
    nature_dir = get_package_share_directory('nature')
    base_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nature_dir, 'launch', 'base.launch.py')),
        launch_arguments={'waypoints_file': waypoints_file,
                          'robot_description': robot_desc,
                          'use_sim_time': 'False',
                          'goal_dist': '5.0',
                          'path_look_ahead': '30.0'}.items()
    )

    # some "global" MAVS parameters
    scene_file = "cube_scene.json"
    veh_file = "mrzr4_tires_low_gear.json"

    env_params = {'env_params':
	        {"rain_rate": 10.0,
                "snow_rate": 0.0}
            }

    mavs_vehicle = launch_ros.actions.Node(
        package='mavs-ros2',
        namespace='mavs',
        executable='mavs_vehicle_node',
        name='mavs_vehicle_node',
        parameters=[
            {'scene_file': scene_file},
            {'rp3d_vehicle_file': veh_file},
            {'soil_strength': 250.0},
            {'surface_type': "dry"},
            {'Initial_X_Position': -50.0},
            {'Initial_Y_Position': 0.0},
            {'Initial_Heading': 0.0},
            {'debug_camera': True},
            {'use_human_driver': False},
            env_params
        ],
        remappings=[
                (('/mavs/cmd_vel'), '/nature/cmd_vel'),
                (('/mavs/odometry_true'), '/nature/odometry')
            ],
        output='screen',
        emulate_tty=True
    )
    mavs_lidar = launch_ros.actions.Node(
            package='mavs-ros2',
            namespace='mavs',
            executable='mavs_lidar_node',
            name='mavs_lidar_node',
            parameters=[
                {'scene_file': scene_file},
                {'offset': [0.0,0.0,1.5]},
                {'orientation': [1.0, 0.0, 0.0, 0.0]},
                {'display': True},
                {'update_rate_hz': 10.0},
                {'vehicle_files': [veh_file]},
                {'lidar_type' : 'OS2'}, #//Options are: HDL-32E', 'HDL-64E', 'M8','OS1', 'OS1-16', 'OS2', 'LMS-291', 'VLP-16', 'RS32
                {'register_points' : True},
                env_params
            ],
            remappings=[
                (('/mavs/lidar'), '/nature/points'),
                (('/mavs/odometry_true'), '/nature/odometry')
            ],
            output='screen',
            emulate_tty=True
        )

    launch_description = LaunchDescription([
        #Node(
        #    package='nature',
        #    executable='nature_sim_test_node',
        #    name='sim_test_node',
        #    output='screen',
        #    parameters=[{'use_sim_time': False}],
        #),
        mavs_vehicle,
        mavs_lidar,
        base_launch
    ])

    return launch_description
