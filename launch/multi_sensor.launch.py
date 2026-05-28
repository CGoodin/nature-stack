import os
 
import launch
import launch_ros.actions
import launch.conditions
import math
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
                          'use_global_path': 'False',
                          'path_look_ahead': '20.0',
                          'max_steer_angle': '0.65',
                          'num_paths': '31',
                          'blanking_distance': '5.0',
                          'grid_res': '0.4',
                          'display_type': 'image',
                          'use_global_fallback': 'False'}.items()
    )
   
    # some "global" MAVS parameters
    #scene_file = "prashant_scene_blocked.json"
    scene_file = "prashant_scene.json"
    veh_file = "army_truck.json"
    actor_file = "pine_tree_actor.json"
 
    #env_params = {'env_params':
    #        {"rain_rate": 0.0,
    #            "snow_rate": 0.0}
    #       }
    
    env_params = {"rain_rate": 0.0, "snow_rate": 10.0}  
 
    mavs_vehicle = launch_ros.actions.Node(
        package='mavs-ros2',
        namespace='mavs',
        executable='mavs_vehicle_node',
        name='mavs_vehicle_node',
        parameters=[
            {'scene_file': scene_file},
            {'rp3d_vehicle_file': veh_file},
            {'actor_files': [actor_file]},
            {'soil_strength': 250.0},
            {'surface_type': "dry"},
            {'Initial_X_Position': -110.0},
            {'Initial_Y_Position': 0.0},
            {'Initial_Heading': 0.0},
            {'debug_camera': False},
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
    
    mavs_follow_camera = Node(
        package='mavs-ros2',
        namespace='mavs',
        executable='mavs_camera_node',
        name='mavs_follow_camera_node',
        parameters=[
            {'scene_file': scene_file},
            {'vehicle_files': [veh_file]},
            {'actor_files': [actor_file]},
            {'camera_type': "rgb"},
            {'num_horizontal_pix': 960},
            {'num_vertical_pix': 540},
            {'horizontal_pixel_plane_size': 0.006222222},
            {'vertical_pixel_plane_size': 0.0035},
            {'focal_length': 0.0035},
            {'sensor_position_mode': "attached"},
            {'offset': [-8.0, 0.0, 2.0]},
            {'orientation': [1.0, 0.0, 0.0, 0.0]},
            {'render_shadows': False},
            {'display': True},
            {'update_rate_hz': 15.0},
            env_params
        ],
        remappings=[
            (('/mavs/odometry_true'), '/nature/odometry')
        ],
        output='screen',
        emulate_tty=True
    )

    mavs_camera = Node(
        package='mavs-ros2',
        namespace='mavs',
        executable='mavs_camera_node',
        name='mavs_camera_node',
        parameters=[
            {'scene_file': scene_file},
            {'vehicle_files': [veh_file]},
            {'actor_files': [actor_file]},
            {'camera_type': "rgb"},
            {'num_horizontal_pix': 768},
            {'num_vertical_pix': 768},
            {'horizontal_pixel_plane_size': 0.0035},
            {'vertical_pixel_plane_size': 0.0035},
            {'focal_length': 0.0019},
            {'sensor_position_mode': "fixed"},
            {'offset': [-25.0, 0.0, 100.0]},
            {'orientation': [0.7071, 0.0, 0.7071, 0.0]},
            {'render_shadows': False},
            {'display': True},
            {'update_rate_hz': 5.0},
            env_params
        ],
        remappings=[
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
            {'offset': [-0.5,0.0,2.5]},
            {'orientation': [1.0, 0.0, 0.0, 0.0]},
            {'display': True},
            {'actor_files': [actor_file]},
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
        
    mavs_actor_manager_node = Node(
        package='mavs-ros2',
        namespace='mavs_actor',
        executable='mavs_actor_manager_node',
        name='mavs_actor_manager_node',
        parameters=[
            {'initial_position':    [-25.0, -10.0, 0.0]},
            {'initial_orientation': [1.0, 0.0, 0.0, 0.0]},
            {'final_position':      [-25.0, -10.0, 0.25]},
            {'final_orientation':   [math.cos(-0.25 * math.pi), math.sin(-0.25 * math.pi), 0.0, 0.0]},
            {'transition_time': 1.0},
            {'trigger': {
                "type":      "x",
                "operator":  ">",
                "threshold": -60.0
            }},
        ],
        remappings=[
            ('/mavs_actor/odometry', '/nature/odometry')
        ],
        output='screen',
        emulate_tty=True
    )
    mavs_aggregator = Node(
        package='mavs-ros2',
        namespace='mavs',
        executable='mavs_pose_aggregator_node',
        name='mavs_aggregator_node',
        parameters=[
            {'num_vehicles': 1, 'num_actors': 1}
        ],
        remappings=[
            ('/mavs000/anim_poses',  '/mavs/anim_poses'),
            ('/mavs000/actor_poses', '/mavs_actor/actor_poses')
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
        mavs_actor_manager_node,
        mavs_aggregator,
        mavs_camera,
        mavs_follow_camera,
        base_launch
    ])
 
    return launch_description