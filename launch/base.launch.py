import os

import launch.conditions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def evaluate_waypoint_parameters(context, *args, **kwargs):
    waypoints_file_path = LaunchConfiguration('waypoints_file').perform(context)
    waypoints_x = "[ ]"
    waypoints_y = "[ ]"
    is_empty_waypoints = not waypoints_file_path
    with open(waypoints_file_path, 'r') as f:
        for line in f.readlines():
            if "waypoints_x" in line:
                waypoints_x = line.split(":")[1]
                is_empty_waypoints = is_empty_waypoints or waypoints_x.replace(' ', '') == '[]'
            if "waypoints_y" in line:
                waypoints_y = line.split(":")[1]
                is_empty_waypoints = is_empty_waypoints or waypoints_y.replace(' ', '') == '[]'

    if is_empty_waypoints:
        waypoints_x = "[ 0.0 ]"
        waypoints_y = "[ 0.0 ]"

    return [
        DeclareLaunchArgument('waypoints_x', description="List of waypoint x coordinates. Will override waypoints_file is specified.", default_value=waypoints_x),
        DeclareLaunchArgument('waypoints_y', description="List of waypoint y coordinates. Will override waypoints_file is specified.", default_value=waypoints_y),
        DeclareLaunchArgument('is_empty_waypoints',
                              description="Parameter set internally to detect if waypoints file empty. ROS2 foxy workaround (https://answers.ros.org/question/396556/what-is-best-practice-for-parameters-which-are-empty-lists-in-ros2/). Do not set manually",
                              default_value=str(is_empty_waypoints).capitalize()),
    ]


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_launch_rviz = LaunchConfiguration("auto_launch_rviz")
    display_type = LaunchConfiguration('display_type')
    waypoints_file = LaunchConfiguration('waypoints_file')
    robot_description = LaunchConfiguration('robot_description')

    rviz_config_path = os.path.join(get_package_share_directory('nature'), 'rviz', 'nature_ros2.rviz')

    launch_description = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('auto_launch_rviz', default_value='False', description="Automatically launch rviz display window"),
        DeclareLaunchArgument('display_type', default_value='rviz', description="Type of display method to use. Values = [rviz, image]"),
        DeclareLaunchArgument('waypoints_file', default_value=os.path.join(get_package_share_directory('nature'), 'config', 'no_waypoints.yaml'), description="Path to waypoint file to use"),
        DeclareLaunchArgument('robot_description', description="URDF robot description contents"),

        # Elevation Grid
        DeclareLaunchArgument('use_elevation', default_value='False', description="Elevation grid - To use elevation or slope value when making occupancy grid based on heightmap."),
        DeclareLaunchArgument('slope_threshold', default_value='0.5', description="Elevation grid - Threshold within which next waypoint selected."),
        DeclareLaunchArgument('grid_height', default_value='200.0', description="Elevation grid - Grid height."),
        DeclareLaunchArgument('grid_width', default_value='200.0', description="Elevation grid - Grid width."),
        DeclareLaunchArgument('grid_llx', default_value='-100.0', description="Elevation grid - X coordinate grid bottom left anchor point."),
        DeclareLaunchArgument('grid_lly', default_value='-100.0', description="Elevation grid - Y coordinate grid bottom left anchor point."),
        DeclareLaunchArgument('grid_res', default_value='0.5', description="Elevation grid - Grid resolution in meters."),
        DeclareLaunchArgument('grid_dilate', default_value='False', description="Elevation grid - Whether or not to apply dilation."),
        DeclareLaunchArgument('grid_dilate_x', default_value='2.0', description="Elevation grid - Amount of dilation in x."),
        DeclareLaunchArgument('grid_dilate_y', default_value='2.0', description="Elevation grid - Amount of dilation in y."),
        DeclareLaunchArgument('grid_dilate_proportion', default_value='0.8', description="Elevation grid - Proportion of original grid cell to dilate with."),
        DeclareLaunchArgument('cull_lidar', default_value='False', description="Elevation grid - Cull lidar points flag based on distance from odometry."),
        DeclareLaunchArgument('cull_lidar_dist', default_value='90.0', description="Elevation grid - Distance used to cull lidar points"),
        DeclareLaunchArgument('use_registered', default_value='True', description="Elevation grid - If true, assumes lidar points are in world coordinates. Else assumes in robot odom coordinates."),
        DeclareLaunchArgument('stitch_lidar_points', default_value='True', description="Elevation grid - If true, lidar scans will be stitched together. Else, each point cloud 2 message will be independent and the grid will be cleared between messages."),
        DeclareLaunchArgument('filter_highest_lidar', default_value='False', description="Elevation grid - If true, the highest point in each cell will be ignored and the second highest will be used for the slope calculations. If false, the highest point will be used."),

        # Global Planner
        DeclareLaunchArgument('goal_dist', default_value='5.0', description="Global planner - Lookahead threshold within which next waypoint selected."),

        # Local Planner
        DeclareLaunchArgument('num_paths', default_value='21', description="Local planner - Number of candidate paths to be generated."),
        DeclareLaunchArgument('path_look_ahead', default_value='30.0', description="Local planner - Planning horizon."),
        DeclareLaunchArgument('vehicle_width', default_value='3.0', description="Local planner - Vehicle width."),
        DeclareLaunchArgument('max_steer_angle', default_value='0.5', description="Local planner - Maximum steer angle in radians. Used to compute rho_max during local planning."),
        DeclareLaunchArgument('dilation_factor', default_value='1', description="Local planner - Dilation to costmap to apply during local planning only."),
        DeclareLaunchArgument('w_c', default_value='0.0', description="Local planner - w_c conformability (minimize curvature) weighting factor"),
        DeclareLaunchArgument('w_s', default_value='0.4', description="Local planner - w_s static safety (avoid collisions) weighting factor"),
        DeclareLaunchArgument('w_d', default_value='0.0', description="Local planner - w_d dynamic safety weighting factor"),
        DeclareLaunchArgument('w_r', default_value='0.2', description="Local planner - w_r rho (minimize rho offset) weighting factor"),
        DeclareLaunchArgument('w_t', default_value='0.0', description="Local planner - w_t segmentation cost weight"),
        DeclareLaunchArgument('use_global_path', default_value='True', description="Local planner - Whether local planner should use path output from global planner for its road centerline or use simple line connecting waypoints"),
        DeclareLaunchArgument('use_blend', default_value='False', description="Local planner - Whether or not to apply blending with adjacent candidate paths during planning"),
        DeclareLaunchArgument('cost_vis', default_value='final', description="Local planner - What type of cost to display on candidate paths: none | final | components | all"),
        DeclareLaunchArgument('cost_vis_text_size', default_value='2.0', description="Local planner - Cost vis text size"),
        DeclareLaunchArgument('ignore_coll_before_dist', default_value='0.0', description="Local planner - Distance before which collisions are ignored in local planner candidate paths."),

        # Pure Pursuit Control
        DeclareLaunchArgument('vehicle_wheelbase', default_value='2.72', description="Pure pursuit controller - vehicle_wheelbase."),
        DeclareLaunchArgument('vehicle_max_steer_angle_degrees', default_value='30.0', description="Pure pursuit controller - max steer angle in degrees."),
        DeclareLaunchArgument('steering_coefficient', default_value='4.5', description="Pure pursuit controller - steering coefficient."),
        DeclareLaunchArgument('vehicle_speed', default_value='5.0', description="Pure pursuit controller - vehicle_speed m/s."),
        DeclareLaunchArgument('throttle_coefficient', default_value='1.0', description="Pure pursuit controller - scale factor for the commanded steering. l.t. 1.0 will make the acceleration less agressive, g.t. 1.0 will make it more agressive"),
        DeclareLaunchArgument('throttle_kp', default_value='0.1129', description="Throttle PID Control - proportional coeff for the PID speed controller"),
        DeclareLaunchArgument('throttle_ki', default_value='0.0', description="Throttle PID Control - integral coeff for the PID speed controller"),
        DeclareLaunchArgument('throttle_kd', default_value='0.0', description="Throttle PID Control - derivative coeff for the PID speed controller"),

        DeclareLaunchArgument('time_to_max_brake', default_value='4.0', description="Time in seconds to go from 0 to maximum braking"),
        DeclareLaunchArgument('time_to_max_throttle', default_value='3.0', description="Time in seconds to go from 0 to maximum throttle"),
        DeclareLaunchArgument('use_feed_forward', default_value='false', description="Set to true to use the feed-forward model in the PID throttle control"),
        DeclareLaunchArgument('ff_a0', default_value='0.2856', description="0th order coeff in the feed-forward model"),
        DeclareLaunchArgument('ff_a1', default_value='0.0321', description="1st order coeff in the feed-forward model"),
        DeclareLaunchArgument('ff_a2', default_value='0.0', description="2nd order coeff in the feed-forward model"),
        DeclareLaunchArgument('max_desired_lateral_g', default_value='0.75', description="Controller will limit the speed to try to keep the lateral g-forces under this amount. In fractional units of 9.806 m/s^2"),

        OpaqueFunction(function=evaluate_waypoint_parameters),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        Node(
            package='nature',
            executable='nature_bot_state_publisher_node',
            name='state_publisher'),
        Node(
            package='nature',
            executable='nature_perception_node',
            name='perception_node',
            output='screen',
            parameters=[{
                'use_elevation': launch.substitutions.LaunchConfiguration('use_elevation'),
                'slope_threshold': launch.substitutions.LaunchConfiguration('slope_threshold'),
                'grid_height': launch.substitutions.LaunchConfiguration('grid_height'),
                'grid_width': launch.substitutions.LaunchConfiguration('grid_width'),
                'grid_llx': launch.substitutions.LaunchConfiguration('grid_llx'),
                'grid_lly': launch.substitutions.LaunchConfiguration('grid_lly'),
                'grid_res': launch.substitutions.LaunchConfiguration('grid_res'),
                'overhead_clearance': 7.0,
                'grid_dilate': launch.substitutions.LaunchConfiguration('grid_dilate'),
                'grid_dilate_x': launch.substitutions.LaunchConfiguration('grid_dilate_x'),
                'grid_dilate_y': launch.substitutions.LaunchConfiguration('grid_dilate_y'),
                'grid_dilate_proportion': launch.substitutions.LaunchConfiguration('grid_dilate_proportion'),
                'cull_lidar': launch.substitutions.LaunchConfiguration('cull_lidar'),
                'cull_lidar_dist': launch.substitutions.LaunchConfiguration('cull_lidar_dist'),
                'warmup_time': 5.0,
                'use_registered': launch.substitutions.LaunchConfiguration('use_registered'),
                'display': display_type,
                'stitch_lidar_points': launch.substitutions.LaunchConfiguration('stitch_lidar_points'),
                'filter_highest_lidar': launch.substitutions.LaunchConfiguration('filter_highest_lidar')
            }],
        ),
        Node(
            package='nature',
            executable='nature_control_node',
            name='vehicle_control_node',
            output='screen',
            parameters=[{
                'vehicle_wheelbase': launch.substitutions.LaunchConfiguration('vehicle_wheelbase'),
                'vehicle_max_steer_angle_degrees': launch.substitutions.LaunchConfiguration('vehicle_max_steer_angle_degrees'),
                'steering_coefficient': launch.substitutions.LaunchConfiguration('steering_coefficient'),
                'vehicle_speed': launch.substitutions.LaunchConfiguration('vehicle_speed'),
                'throttle_coefficient': launch.substitutions.LaunchConfiguration('throttle_coefficient'),
                'throttle_kp': launch.substitutions.LaunchConfiguration('throttle_kp'),
                'throttle_ki': launch.substitutions.LaunchConfiguration('throttle_ki'),
                'throttle_kd': launch.substitutions.LaunchConfiguration('throttle_kd'),
                'time_to_max_brake': launch.substitutions.LaunchConfiguration('time_to_max_brake'),
                'max_desired_lateral_g': launch.substitutions.LaunchConfiguration('max_desired_lateral_g'),
            }],
        ),
        Node(
            package='nature',
            executable='nature_global_path_node',
            name='nature_global_path_node',
            output='screen',
            parameters=[{
                'goal_dist': launch.substitutions.LaunchConfiguration('goal_dist'),
                'global_lookahead': 75.0,
                'shutdown_behavior': 2,
                'display': display_type,
                '/waypoints_x': launch.substitutions.LaunchConfiguration('waypoints_x'),
                '/waypoints_y': launch.substitutions.LaunchConfiguration('waypoints_y'),
                '/is_empty_waypoints': launch.substitutions.LaunchConfiguration('is_empty_waypoints'),
            }],
        ),
        Node(
            package='nature',
            executable='nature_local_planner_node',
            name='local_planner_node',
            output='screen',
            parameters=[{
                'path_look_ahead': launch.substitutions.LaunchConfiguration('path_look_ahead'),
                'vehicle_width': launch.substitutions.LaunchConfiguration('vehicle_width'),
                'num_paths': launch.substitutions.LaunchConfiguration('num_paths'),
                'max_steer_angle': launch.substitutions.LaunchConfiguration('max_steer_angle'),
                'output_path_step': 0.5,
                'path_integration_step': 0.35,
                'dilation_factor': launch.substitutions.LaunchConfiguration('dilation_factor'),
                'w_c': launch.substitutions.LaunchConfiguration('w_c'),
                'w_s': launch.substitutions.LaunchConfiguration('w_s'),
                'w_d': launch.substitutions.LaunchConfiguration('w_d'),
                'w_r': launch.substitutions.LaunchConfiguration('w_r'),
                'w_t': launch.substitutions.LaunchConfiguration('w_t'),
                'rate': 50.0,
                'trim_path': True,
                'use_global_path': launch.substitutions.LaunchConfiguration('use_global_path'),
                'use_blend': launch.substitutions.LaunchConfiguration('use_blend'),
                'cost_vis': launch.substitutions.LaunchConfiguration('cost_vis'),
                'cost_vis_text_size': launch.substitutions.LaunchConfiguration('cost_vis_text_size'),
                'ignore_coll_before_dist': launch.substitutions.LaunchConfiguration('ignore_coll_before_dist'),
                'display': display_type,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=launch.conditions.IfCondition(auto_launch_rviz),
            arguments=["-d", rviz_config_path])
    ])

    return launch_description
