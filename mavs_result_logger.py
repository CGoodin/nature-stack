#!/usr/bin/env python3
# Jordon Jasper 6/3/2025
# Note: The purpose of this file is to automatically log the results of
# MAVS simulation run in ROS2. It subscribes to odometry data and saves
# a structured JSON file containing metrics useful for performance evaluation.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import sqrt
from statistics import mode, StatisticsError
import json
import os


class MAVSResultLogger(Node):
    def __init__(self):
        super().__init__('mavs_result_logger')

        # Set result file directory
        self.declare_parameter('result_file', 'PARAMETER_NOT_SET')
        base_path = self.get_parameter('result_file').get_parameter_value().string_value
        if not base_path or base_path == "PARAMETER_NOT_SET":
            base_path = '/home/jdj688/mavs_ros2/src/nature/results'
            self.get_logger().info(f"[Init] No result_file param set. Using default directory: {base_path}")
        else:
            self.get_logger().info(f"[Init] Result file base path: {base_path}")
        os.makedirs(base_path, exist_ok=True)

        # Auto-increment run ID
        self.test_id, self.result_file = self.generate_unique_result_file(base_path)

        # Subscribe to odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/nature/odometry',
            self.odom_callback,
            10
        )
        self.get_logger().info("[Init] Subscribed to /nature/odometry (expecting nav_msgs/msg/Odometry)")

        # Variables
        self.start_location = {'Initial_X_Position': -50.0, 'Initial_Y_Position': 0.0, 'Initial_Z_Position': 0.0}
        self.final_location = None
        self.goal_coords = {'x': 48.7, 'y': 0.0, 'z': 0.0}
        self.goal_threshold = 2.0

        self.elapsed = 0
        self.timeout = 60
        self.start_time = None
        self.time_taken = None
        self.odom_received = False
        self.goal_reached = False
        self.speed_mph = 0.0
        self.prev_position = None
        self.total_distance = 0.0
        self.speed_history = []

        # Timers
        self.timeout_timer = self.create_timer(1.0, self.timeout_check)
        self.debug_timer = self.create_timer(3.0, self.debug_log)

    def generate_unique_result_file(self, base_path):
        counter = 1
        while True:
            test_id = f"run_{counter}"
            filename = os.path.join(base_path, f"{test_id}.json")
            if not os.path.exists(filename):
                return test_id, filename
            counter += 1

    def odom_callback(self, msg):
        if not self.odom_received:
            self.get_logger().info("[Odom Callback] First odometry message received.")
        self.odom_received = True

        if self.start_time is None:
            self.start_time = self.get_clock().now()
            self.get_logger().info("[Odom Callback] Start time recorded.")

        pos = msg.pose.pose.position
        self.final_location = pos

        # Update distance traveled
        if self.prev_position:
            dx = pos.x - self.prev_position.x
            dy = pos.y - self.prev_position.y
            dz = pos.z - self.prev_position.z
            dist = sqrt(dx**2 + dy**2 + dz**2)
            self.total_distance += dist
        self.prev_position = pos

        # Update time
        current_time = self.get_clock().now()
        self.time_taken = (current_time - self.start_time).nanoseconds / 1e9

        # Update and store speed
        speed_mps = msg.twist.twist.linear.x
        self.speed_mph = round(speed_mps * 2.23694, 2)
        self.speed_history.append(self.speed_mph)

        self.get_logger().info(f"[Odom Callback] Speed: {self.speed_mph:.2f} mph")
        self.get_logger().info(f"[Odom Callback] Pos: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")
        self.get_logger().info(f"[Odom Callback] Time taken: {self.time_taken:.2f}s")

        if self.is_goal_reached(pos, self.goal_coords, self.goal_threshold):
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info("[Odom Callback] Goal reached!")
                self.save_results()
                rclpy.shutdown()

    def timeout_check(self):
        self.elapsed += 1
        if self.elapsed >= self.timeout:
            self.get_logger().warn("[Timeout] Timeout reached!")
            if not self.odom_received:
                self.get_logger().warn("[Timeout] No odometry data was ever received.")
            else:
                self.get_logger().info("[Timeout] Final odometry was received.")
            self.save_results()
            rclpy.shutdown()

    def debug_log(self):
        self.get_logger().info(f"[Debug] Time elapsed: {self.elapsed}s | Odom received: {self.odom_received} | Goal reached: {self.goal_reached}")
        if self.time_taken:
            self.get_logger().info(f"[Debug] Time taken: {self.time_taken:.2f}s")
            self.get_logger().info(f"[Debug] Distance traveled: {self.total_distance:.2f} m")

    def is_goal_reached(self, final, goal, threshold):
        dx = final.x - goal['x']
        dy = final.y - goal['y']
        dz = final.z - goal['z']
        return sqrt(dx ** 2 + dy ** 2 + dz ** 2) <= threshold

    def save_results(self):
        if not self.final_location:
            self.get_logger().warn("[Save] No final location. Saving fallback result.")
            final = {'x': None, 'y': None, 'z': None}
            goal_reached = False
        else:
            final = {
                'x': self.final_location.x,
                'y': self.final_location.y,
                'z': self.final_location.z
            }
            goal_reached = self.goal_reached

        # Calculate mode of speed history
        if self.speed_history:
            try:
                mode_speed = mode(self.speed_history)
            except StatisticsError:
                mode_speed = self.speed_history[-1]  # fallback: last known speed
        else:
            mode_speed = 0.0

        result = {
            'test_id': self.test_id,
            'time_taken': self.time_taken,
            'initial_location': self.start_location,
            'goal_location': self.goal_coords,
            'final_location': final,
            'goal_reached': goal_reached,
            'average_speed': mode_speed
        }

        with open(self.result_file, 'w') as f:
            json.dump(result, f, indent=2)

        self.get_logger().info(f"[Save] Results saved to {self.result_file}")
        self.get_logger().info(f"[Save] Goal reached: {goal_reached}")
        os._exit(0) #Need this line of code to stop execution after goal is reached
        

def main(args=None):
    rclpy.init(args=args)
    logger = MAVSResultLogger()
    rclpy.spin(logger)
    logger.destroy_node()
    rclpy.shutdown()
    os._exit(0)

if __name__ == '__main__':
    main()

