import os
import launch.conditions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

def generate_launch_description():

    launch_description = LaunchDescription([
       
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
        )
       
    ])
    return launch_description
