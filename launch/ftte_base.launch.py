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
        DeclareLaunchArgument('grid_height', default_value='400.0', description="Elevation grid - Grid height."),
        DeclareLaunchArgument('grid_width', default_value='400.0', description="Elevation grid - Grid width."),
        DeclareLaunchArgument('grid_llx', default_value='-200.0', description="Elevation grid - X coordinate grid bottom left anchor point."),
        DeclareLaunchArgument('grid_lly', default_value='-200.0', description="Elevation grid - Y coordinate grid bottom left anchor point."),
        DeclareLaunchArgument('grid_res', default_value='0.5', description="Elevation grid - Grid resolution in meters."),
        DeclareLaunchArgument('grid_dilate', default_value='False', description="Elevation grid - Whether or not to apply dilation."),
        DeclareLaunchArgument('grid_dilate_x', default_value='1.0', description="Elevation grid - Amount of dilation in x."),
        DeclareLaunchArgument('grid_dilate_y', default_value='1.0', description="Elevation grid - Amount of dilation in y."),
        DeclareLaunchArgument('grid_dilate_proportion', default_value='0.8', description="Elevation grid - Proportion of original grid cell to dilate with."),
        DeclareLaunchArgument('cull_lidar', default_value='False', description="Elevation grid - Cull lidar points flag based on distance from odometry."),
        DeclareLaunchArgument('cull_lidar_dist', default_value='90.0', description="Elevation grid - Distance used to cull lidar points"),
        DeclareLaunchArgument('blanking_distance', default_value='2.0', description="Elevation grid - Distances less than this are ignored. Keeps points from the vehicle from being reported"),
        DeclareLaunchArgument('use_registered', default_value='True', description="Elevation grid - If true, assumes lidar points are in world coordinates. Else assumes in robot odom coordinates."),
        DeclareLaunchArgument('stitch_lidar_points', default_value='False', description="Elevation grid - If true, lidar scans will be stitched together. Else, each point cloud 2 message will be independent and the grid will be cleared between messages."),
        DeclareLaunchArgument('filter_highest_lidar', default_value='False', description="Elevation grid - If true, the highest point in each cell will be ignored and the second highest will be used for the slope calculations. If false, the highest point will be used."),

        # Global Planner
        DeclareLaunchArgument('goal_dist', default_value='5.0', description="Global planner - Lookahead threshold within which next waypoint selected."),

        # Local Planner
        DeclareLaunchArgument('num_paths', default_value='15', description="Local planner - Number of candidate paths to be generated."),
        DeclareLaunchArgument('path_look_ahead', default_value='5.0', description="Local planner - Planning horizon."),
        DeclareLaunchArgument('vehicle_width', default_value='3.0', description="Local planner - Vehicle width."),
        DeclareLaunchArgument('max_steer_angle', default_value='0.5', description="Local planner - Maximum steer angle in radians. Used to compute rho_max during local planning."),
        DeclareLaunchArgument('dilation_factor', default_value='1', description="Local planner - Dilation to costmap to apply during local planning only."),
        DeclareLaunchArgument('w_c', default_value='0.02', description="Local planner - w_c conformability (minimize curvature) weighting factor"),
        DeclareLaunchArgument('w_s', default_value='1.0', description="Local planner - w_s static safety (avoid collisions) weighting factor"),
        DeclareLaunchArgument('w_d', default_value='0.0', description="Local planner - w_d dynamic safety weighting factor"),
        DeclareLaunchArgument('w_r', default_value='0.05', description="Local planner - w_r rho (minimize rho offset) weighting factor"),
        DeclareLaunchArgument('w_t', default_value='0.00', description="Local planner - w_t segmentation cost weight"),
        DeclareLaunchArgument('use_global_path', default_value='True', description="Local planner - Whether local planner should use path output from global planner for its road centerline or use simple line connecting waypoints"),
        DeclareLaunchArgument('use_global_fallback', default_value='False', description="Local planner - If the local planner fails to find a valid path, should it just republish the global path? If False, it will publish an empty path"),
        DeclareLaunchArgument('use_blend', default_value='True', description="Local planner - Whether or not to apply blending with adjacent candidate paths during planning"),
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

        # FTTE Node
        DeclareLaunchArgument('use_registered_points', default_value='False', description="FTTE - If true, uses registered (world-frame) lidar points."),
        DeclareLaunchArgument('use_loam', default_value='False', description="FTTE - If true, uses LOAM-based odometry."),
        DeclareLaunchArgument('map_res', default_value='0.5', description="FTTE - Map resolution in meters."),
        DeclareLaunchArgument('map_width', default_value='100.0', description="FTTE - Map width in meters."),
        DeclareLaunchArgument('map_length', default_value='100.0', description="FTTE - Map length in meters."),
        DeclareLaunchArgument('fixed_map', default_value='True', description="FTTE - If true, map is fixed and does not scroll with the vehicle."),
        DeclareLaunchArgument('vehicle_mass', default_value='1600.0', description="FTTE - Vehicle mass in kg."),
        DeclareLaunchArgument('vehicle_bumper_height', default_value='0.45', description="FTTE - Height of the vehicle bumper in meters."),
        DeclareLaunchArgument('vehicle_tire_radius', default_value='0.36', description="FTTE - Radius of the vehicle tires in meters."),
        DeclareLaunchArgument('vehicle_vci1', default_value='10.0', description="FTTE - Vehicle cone index 1 (VCI1), a measure of mobility on soft soils."),
        DeclareLaunchArgument('vehicle_max_slope', default_value='0.55', description="FTTE - Maximum climbable slope in radians."),
        DeclareLaunchArgument('vehicle_roof_height', default_value='3.5', description="FTTE - Height of the vehicle roof in meters."),
        DeclareLaunchArgument('default_traversability', default_value='0.6', description="FTTE - Default traversability score assigned to unmeasured cells (0.0-1.0)."),
        DeclareLaunchArgument('slope_coeff', default_value='0.0', description="FTTE - Slope traversability weighting coefficient."),
        DeclareLaunchArgument('slope_exponent', default_value='2.0', description="FTTE - Exponent applied to slope cost term."),
        DeclareLaunchArgument('soil_coeff', default_value='0.0', description="FTTE - Soil traversability weighting coefficient."),
        DeclareLaunchArgument('soil_exponent', default_value='1.0', description="FTTE - Exponent applied to soil cost term."),
        DeclareLaunchArgument('veg_coeff', default_value='1.0', description="FTTE - Vegetation traversability weighting coefficient."),
        DeclareLaunchArgument('veg_exponent', default_value='1.0', description="FTTE - Exponent applied to vegetation cost term."),
        DeclareLaunchArgument('roughness_coeff', default_value='1.0', description="FTTE - Roughness traversability weighting coefficient."),
        DeclareLaunchArgument('roughness_exponent', default_value='1.0', description="FTTE - Exponent applied to roughness cost term."),
        DeclareLaunchArgument('use_planes', default_value='False', description="FTTE - If true, uses plane fitting for terrain estimation."),
        DeclareLaunchArgument('show_timing', default_value='False', description="FTTE - If true, outputs timing diagnostics."),
        DeclareLaunchArgument('show_traversability', default_value='False', description="FTTE - If true, publishes traversability visualization."),
        DeclareLaunchArgument('show_confidence', default_value='False', description="FTTE - If true, publishes confidence map visualization."),
        DeclareLaunchArgument('show_ground', default_value='False', description="FTTE - If true, publishes ground point visualization."),
        DeclareLaunchArgument('show_roughness', default_value='False', description="FTTE - If true, publishes roughness map visualization."),
        DeclareLaunchArgument('show_veg', default_value='False', description="FTTE - If true, publishes vegetation map visualization."),
        DeclareLaunchArgument('show_slope', default_value='False', description="FTTE - If true, publishes slope map visualization."),

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
        # Node(
        #     package='nature',
        #     executable='nature_perception_node',
        #     name='perception_node',
        #     output='screen',
        #     parameters=[{
        #         'use_elevation': launch.substitutions.LaunchConfiguration('use_elevation'),
        #         'slope_threshold': launch.substitutions.LaunchConfiguration('slope_threshold'),
        #         'grid_height': launch.substitutions.LaunchConfiguration('grid_height'),
        #         'grid_width': launch.substitutions.LaunchConfiguration('grid_width'),
        #         'grid_llx': launch.substitutions.LaunchConfiguration('grid_llx'),
        #         'grid_lly': launch.substitutions.LaunchConfiguration('grid_lly'),
        #         'grid_res': launch.substitutions.LaunchConfiguration('grid_res'),
        #         'overhead_clearance': 7.0,
        #         'grid_dilate': launch.substitutions.LaunchConfiguration('grid_dilate'),
        #         'grid_dilate_x': launch.substitutions.LaunchConfiguration('grid_dilate_x'),
        #         'grid_dilate_y': launch.substitutions.LaunchConfiguration('grid_dilate_y'),
        #         'grid_dilate_proportion': launch.substitutions.LaunchConfiguration('grid_dilate_proportion'),
        #         'cull_lidar': launch.substitutions.LaunchConfiguration('cull_lidar'),
        #         'cull_lidar_dist': launch.substitutions.LaunchConfiguration('cull_lidar_dist'),
        #         'blanking_distance': launch.substitutions.LaunchConfiguration('blanking_distance'),
        #         'warmup_time': 5.0,
        #         'use_registered': launch.substitutions.LaunchConfiguration('use_registered'),
        #         'display': display_type,
        #         'stitch_lidar_points': launch.substitutions.LaunchConfiguration('stitch_lidar_points'),
        #         'filter_highest_lidar': launch.substitutions.LaunchConfiguration('filter_highest_lidar')
        #     }],
        #),
        Node(
            package='nature',
            executable='nature_ftte_node',
            name='nature_ftte_node',
            output='screen',
            parameters=[{
                'use_registered_points': launch.substitutions.LaunchConfiguration('use_registered_points'),
                'use_loam': launch.substitutions.LaunchConfiguration('use_loam'),
                'map_res': launch.substitutions.LaunchConfiguration('map_res'),
                'map_width': launch.substitutions.LaunchConfiguration('map_width'),
                'map_length': launch.substitutions.LaunchConfiguration('map_length'),
                'fixed_map': launch.substitutions.LaunchConfiguration('fixed_map'),
                'vehicle_mass': launch.substitutions.LaunchConfiguration('vehicle_mass'),
                'vehicle_bumper_height': launch.substitutions.LaunchConfiguration('vehicle_bumper_height'),
                'vehicle_tire_radius': launch.substitutions.LaunchConfiguration('vehicle_tire_radius'),
                'vehicle_vci1': launch.substitutions.LaunchConfiguration('vehicle_vci1'),
                'vehicle_max_slope': launch.substitutions.LaunchConfiguration('vehicle_max_slope'),
                'vehicle_roof_height': launch.substitutions.LaunchConfiguration('vehicle_roof_height'),
                'default_traversability': launch.substitutions.LaunchConfiguration('default_traversability'),
                'slope_coeff': launch.substitutions.LaunchConfiguration('slope_coeff'),
                'slope_exponent': launch.substitutions.LaunchConfiguration('slope_exponent'),
                'soil_coeff': launch.substitutions.LaunchConfiguration('soil_coeff'),
                'soil_exponent': launch.substitutions.LaunchConfiguration('soil_exponent'),
                'veg_coeff': launch.substitutions.LaunchConfiguration('veg_coeff'),
                'veg_exponent': launch.substitutions.LaunchConfiguration('veg_exponent'),
                'roughness_coeff': launch.substitutions.LaunchConfiguration('roughness_coeff'),
                'roughness_exponent': launch.substitutions.LaunchConfiguration('roughness_exponent'),
                'use_planes': launch.substitutions.LaunchConfiguration('use_planes'),
                'show_timing': launch.substitutions.LaunchConfiguration('show_timing'),
                'show_traversability': launch.substitutions.LaunchConfiguration('show_traversability'),
                'show_confidence': launch.substitutions.LaunchConfiguration('show_confidence'),
                'show_ground': launch.substitutions.LaunchConfiguration('show_ground'),
                'show_roughness': launch.substitutions.LaunchConfiguration('show_roughness'),
                'show_veg': launch.substitutions.LaunchConfiguration('show_veg'),
                'show_slope': launch.substitutions.LaunchConfiguration('show_slope'),
                'grid_height': launch.substitutions.LaunchConfiguration('grid_height'),
                'grid_width': launch.substitutions.LaunchConfiguration('grid_width'),
                'grid_llx': launch.substitutions.LaunchConfiguration('grid_llx'),
                'grid_lly': launch.substitutions.LaunchConfiguration('grid_lly'),
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
                'use_global_fallback': launch.substitutions.LaunchConfiguration('use_global_fallback'),
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
