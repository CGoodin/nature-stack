import os

import launch
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

    launch_description = LaunchDescription([
        Node(
            package='nature',
            executable='nature_sim_test_node',
            name='sim_test_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        base_launch
    ])

    return launch_description
