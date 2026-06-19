#include <iostream>
#include "std_msgs/msg/bool.hpp"
#include "nature/node/ros_types.h"
#include "nature/node/node_proxy.h"
#include "nature/control/pure_pursuit_controller.h"
#include "nature/control/tinyfiledialogs.h"

nature::msg::Path control_msg;
nature::msg::Odometry state;
int current_run_state = -1;   
bool shutdown_condition = false;
double mrzr_speedometer = 0.0;
bool speedometer_rcvd = false;
double mrzr_steering = 0.0;
bool path_rcvd = false;
double current_heading = 0.0;

double HeadingFromQuaternion(double qw, double qx, double qy, double qz) {
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}

void OdometryCallback(nature::msg::OdometryPtr rcv_state) {
  state = *rcv_state; 
  current_heading = HeadingFromQuaternion(state.pose.pose.orientation.w, state.pose.pose.orientation.x, state.pose.pose.orientation.y, state.pose.pose.orientation.z);
}

void SpeedCallback(nature::msg::Float64Ptr rcv_speed) {
  mrzr_speedometer = rcv_speed->data;
  speedometer_rcvd = true; 
}

void SteeringCallback(nature::msg::Float64Ptr rcv_steering) {
  mrzr_steering = rcv_steering->data;
}

void PathCallback(nature::msg::PathPtr rcv_control){
  control_msg.poses = rcv_control->poses;
  control_msg.header = rcv_control->header;
  path_rcvd = true;
}

void StateCallback(nature::msg::Int32Ptr rcv_state){
  current_run_state = rcv_state->data;
  if (current_run_state==2)shutdown_condition = true;
}

int main(int argc, char *argv[]){
  auto n = nature::node::init_node(argc,argv,"nature_control_node");
  auto dc_pub = n->create_publisher<nature::msg::Twist>("nature/cmd_vel",1);
  auto stop_alert_pub = n->create_publisher<std_msgs::msg::Bool>("/nature/vehicle_stop_alert", 1);
  auto path_sub = n->create_subscription<nature::msg::Path>("nature/local_path",1, PathCallback);
  auto state_sub = n->create_subscription<nature::msg::Odometry>("nature/odometry",1, OdometryCallback);
  auto control_sub = n->create_subscription<nature::msg::Int32>("nature/state",1,StateCallback);
  auto speed_sub = n->create_subscription<nature::msg::Float64>("mrzr_velocity",1,SpeedCallback);
  auto steering_sub = n->create_subscription<nature::msg::Float64>("mrzr_steering",1,SteeringCallback);

  nature::control::PurePursuitController controller;

  float time_to_max_throttle = 3.0f; 
  float ff_a0, ff_a1, ff_a2;
  bool use_feed_forward;
  float wheelbase, steer_angle, vehicle_speed, steering_coeff, throttle_coeff, time_to_max_brake, time_to_max_steering;
  float throttle_kp, throttle_ki, throttle_kd, max_desired_lateral_g;
  std::string display;
  bool request_approval;
  
  n->get_parameter("~vehicle_wheelbase", wheelbase, 2.6f);
  n->get_parameter("~vehicle_max_steer_angle_degrees", steer_angle, 25.0f);
  n->get_parameter("~vehicle_speed", vehicle_speed, 5.0f);
  n->get_parameter("~request_approval", request_approval, false);
  n->get_parameter("~steering_coefficient", steering_coeff, 2.0f);
  n->get_parameter("~throttle_coefficient", throttle_coeff, 1.0f);
  n->get_parameter("~time_to_max_brake", time_to_max_brake, 4.0f);
  n->get_parameter("~time_to_max_throttle", time_to_max_throttle, 3.0f);
  n->get_parameter("~time_to_max_steering", time_to_max_steering, 3.0f);
  n->get_parameter("~ff_a0", ff_a0, 0.0402f);
  n->get_parameter("~ff_a1", ff_a1, 0.0814f);
  n->get_parameter("~ff_a2", ff_a2, -0.0023f);
  n->get_parameter("~use_feed_forward", use_feed_forward, true);
  n->get_parameter("~throttle_kp", throttle_kp, 0.462f);
  n->get_parameter("~throttle_ki", throttle_ki, 0.222f);
  n->get_parameter("~throttle_kd", throttle_kd, 0.24f);
  n->get_parameter("~display", display, std::string("none"));
  n->get_parameter("~max_desired_lateral_g", max_desired_lateral_g, 0.75f);

  bool turn_off_velocity_overshoot_corrector;
  n->get_parameter("~turn_off_velocity_overshoot_corrector", turn_off_velocity_overshoot_corrector, false);

  bool skid_steered;
  n->get_parameter("~skid_steered", skid_steered, false);
  float skid_kl, skid_kt;
  n->get_parameter("~skid_kl", skid_kl, 1.0f);
  n->get_parameter("~skid_kt", skid_kt, 1.0f);
  
  if (skid_steered){
    controller.IsSkidSteered(true);
    controller.SetSkidSteerParams(skid_kl, skid_kt);
  }
  else{
    controller.SetSteeringParam(steering_coeff);
    controller.SetThrottleCoeff(throttle_coeff);
    controller.SetWheelbase(wheelbase);
    controller.SetMaxSteering(steer_angle*3.14159 / 180.0);
    controller.SetSpeedControllerParams(throttle_kp, throttle_ki, throttle_kd);
  }

  if (use_feed_forward && !skid_steered){
    controller.GetPidSpeedController()->SetUseFeedForward(true);
    controller.GetPidSpeedController()->SetForwardModelParams(ff_a0, ff_a1, ff_a2);
  }
  
  controller.SetDesiredSpeed(vehicle_speed);
  if (turn_off_velocity_overshoot_corrector){
    controller.GetPidSpeedController()->SetOvershootLimiter(false);
  }

  bool display_rviz = display == "rviz";
  auto next_waypoint_pub = display_rviz ? n->create_publisher<nature::msg::PointStamped>("nature/control_next_waypoint", 1) : nullptr;

  float rate = 100.0f;
  float dt = 1.0f/rate;
  float brake_step = dt/time_to_max_brake;
  float max_throttle_step = dt/time_to_max_throttle;
  float current_brake_value = 0.0f;
  float current_throttle_value = 0.0f;
  float current_steering_value = 0.0f;

  bool is_uturning = false;
  
  nature::node::Rate r(rate);
  nature::utils::vec2 goal;

  float stopped_timer = 0.0f;
  bool stop_warning_published = false;
  bool has_moved = false;

  while (nature::node::ok()){
    nature::msg::Twist dc;
    bool time_to_quit = false;

    float vel = 0.0f;
    if (speedometer_rcvd){
      vel = mrzr_speedometer;
      current_steering_value = mrzr_steering;
    }
    else{
        double look_to_x = cos(current_heading);
        double look_to_y = sin(current_heading);
        vel = state.twist.twist.linear.x * look_to_x + state.twist.twist.linear.y * look_to_y;
    }

    if (!has_moved && std::abs(vel) > 0.5f) {
        has_moved = true;
    }

    controller.SetVehicleState(state);
    controller.SetVehicleSpeed(vel);

    int num_path_poses = control_msg.poses.size();

    // Verify path orientation relative to vehicle heading
    if (num_path_poses > 3 && !is_uturning && std::abs(vel) < 0.5f) {
        
        int target_idx = std::min(5, num_path_poses - 1);
        double tx = control_msg.poses[target_idx].pose.position.x - state.pose.pose.position.x;
        double ty = control_msg.poses[target_idx].pose.position.y - state.pose.pose.position.y;
        
        double hx = cos(current_heading);
        double hy = sin(current_heading);
        double dot_product = (tx * hx) + (ty * hy);
        
        // Initiate U-Turn sequence if local path originates behind the vehicle
        if (dot_product < -0.1) { 
            RCLCPP_WARN(n->get_logger(), "Path is behind vehicle! Bypassing splines for CCW U-Turn.");
            is_uturning = true;
        }
    }

    // Execute hardcoded CCW U-Turn maneuver
    if (is_uturning) {
        int target_idx = std::min(5, num_path_poses - 1);
        double tx = control_msg.poses[target_idx].pose.position.x - state.pose.pose.position.x;
        double ty = control_msg.poses[target_idx].pose.position.y - state.pose.pose.position.y;
        double target_angle = atan2(ty, tx);

        // Calculate heading error
        double angle_error = target_angle - current_heading;
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

        // Terminate maneuver when heading is within ~11 degree tolerance!
        if (std::abs(angle_error) < 0.2) {
            RCLCPP_INFO(n->get_logger(), "U-Turn Complete! Resuming spline following.");
            is_uturning = false;
            dc.linear.x = 0.0;
            dc.linear.y = 0.0; 
            dc.angular.z = 0.0;
        } else { 
            dc.linear.x = 0.35;  
            dc.linear.y = 0.0;   
            dc.angular.z = 1.0; 
        }

        // Bypass standard pure pursuit evaluation during manual maneuver
        dc_pub->publish(dc);
        n->spin_some();
        r.sleep();
        continue; 
    }
    
    if (num_path_poses <= 1) {
        dc.linear.x = 0.0f;
        dc.angular.z = 0.0f;

        if (vel > 0.1f) {
            dc.linear.y = -1.0f;  
        } else {
            dc.linear.y = 0.0f;   
        }

        if (has_moved && std::abs(vel) < 0.1f) {
            stopped_timer += dt; 
            
            if (stopped_timer >= 10.0f && !stop_warning_published) {
                RCLCPP_WARN(n->get_logger(), "Vehicle has been stopped for 10 seconds. Publishing alert!");
                std_msgs::msg::Bool alert_msg;
                alert_msg.data = true;
                stop_alert_pub->publish(alert_msg);
                stop_warning_published = true; 
            }
        } else {
            stopped_timer = 0.0f;
        }
    }
    else {
        if (stop_warning_published) {
            std_msgs::msg::Bool alert_msg;
            alert_msg.data = false;
            stop_alert_pub->publish(alert_msg);
        }

        stopped_timer = 0.0f;
        stop_warning_published = false;

        if (shutdown_condition) {  
            controller.SetDesiredSpeed(0.0f);
            if (vel<0.1f)time_to_quit = true;
            dc = controller.GetDcFromTraj(control_msg, goal);
            dc.linear.y *= 2.0; 
        }
        else if (current_run_state==0) {    
            float desired_velocity = vehicle_speed;
            controller.SetDesiredSpeed(desired_velocity);
            dc = controller.GetDcFromTraj(control_msg, goal);
        }
        else if (current_run_state==-1 || current_run_state==1) {
            controller.SetDesiredSpeed(0.0f);
            dc = controller.GetDcFromTraj(control_msg, goal);
            if (current_run_state==-1)dc.linear.x = 0.0f;
        }
        else if (current_run_state==3) {
            dc.linear.x = 0.0f;
            dc.linear.y = 0.0f;
            dc.angular.z = 0.0f;
            time_to_quit = true;
        }
    }
    
    if (!skid_steered){
      if (dc.linear.y!=0.0){
        if (current_brake_value>dc.linear.y){
          dc.linear.y = current_brake_value - brake_step;
          if (dc.linear.y<-1.0)dc.linear.y = -1.0;
          if (dc.linear.y>0.0)dc.linear.y = 0.0;
        }
        dc.linear.x = 0.0f;
      }
      if (dc.linear.x-current_throttle_value > max_throttle_step){
        dc.linear.x = current_throttle_value + max_throttle_step;
      }
    }
    else{
      dc.linear.x = std::max(std::min(dc.linear.x, 1.0),0.0);
      if (dc.linear.x>0.0f)dc.linear.y = 0.0f;
    }

    dc_pub->publish(dc);
    current_brake_value = dc.linear.y;
    current_throttle_value = dc.linear.x;
    current_steering_value = dc.angular.z; 

    if (time_to_quit) {
        dc.linear.x = 0.0;
        dc.linear.y = 0.0;
        dc.angular.z = 0.0;
        dc_pub->publish(dc);
        break;
    }
    if(display_rviz){
      nature::msg::PointStamped next_waypoint_msg;
      next_waypoint_msg.point.x = goal.x;
      next_waypoint_msg.point.y = goal.y;
      next_waypoint_msg.point.z = state.pose.pose.position.z;
      next_waypoint_msg.header.frame_id = "map";
      next_waypoint_msg.header.stamp = n->get_stamp();
      next_waypoint_pub->publish(next_waypoint_msg);
    }
    n->spin_some();
    r.sleep();
  }
  return 0;
}