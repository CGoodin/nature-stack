/**
 * \file avt_341_control_node.cpp
 *
 * ROS node to subsribe to a trajectory message and 
 * convert it to a driving command using the pure-pursuit algorithm
 * 
 * \author Chris Goodin
 *
 * \contact cgoodin@cavs.msstate.edu
 * 
 * \date 7/13/2018
 */

#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
//avt_341 includes
#include "avt_341/control/pure_pursuit_controller.h"

avt_341::msg::Path control_msg;
avt_341::msg::Odometry state;
int current_run_state = -1;   // startup state

void OdometryCallback(avt_341::msg::OdometryPtr rcv_state) {
	state = *rcv_state; 
}

void PathCallback(avt_341::msg::PathPtr rcv_control){
  control_msg.poses = rcv_control->poses;
  control_msg.header = rcv_control->header;
}

void StateCallback(avt_341::msg::Int32Ptr rcv_state){
  current_run_state = rcv_state->data;
}

int main(int argc, char *argv[]){
  auto n = avt_341::node::init_node(argc,argv,"avt_341_control_node");

  auto dc_pub = n->create_publisher<avt_341::msg::Twist>("avt_341/cmd_vel",1);

  auto path_sub = n->create_subscription<avt_341::msg::Path>("avt_341/local_path",1, PathCallback);

  auto state_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry",1, OdometryCallback);

  auto control_sub = n->create_subscription<avt_341::msg::Int32>("avt_341/state",1,StateCallback);


  avt_341::control::PurePursuitController controller;
	// Set controller parameters
	float wheelbase, steer_angle, vehicle_speed, steering_coeff, throttle_coeff, time_to_max_brake;
  float throttle_kp, throttle_ki, throttle_kd;
	std::string display;
	n->get_parameter("~vehicle_wheelbase", wheelbase, 2.6f);
  n->get_parameter("~vehicle_max_steer_angle_degrees", steer_angle, 25.0f);
  n->get_parameter("~vehicle_speed", vehicle_speed, 5.0f);
  n->get_parameter("~steering_coefficient", steering_coeff, 2.0f);
  n->get_parameter("~throttle_coefficient", throttle_coeff, 1.0f);
  n->get_parameter("~time_to_max_brake", time_to_max_brake, 4.0f);
  n->get_parameter("~throttle_kp", throttle_kp, 0.09f);
  n->get_parameter("~throttle_ki", throttle_ki, 0.01f);
  n->get_parameter("~throttle_kd", throttle_kd, 0.16f);
  n->get_parameter("~display", display, std::string("none"));
  controller.SetSteeringParam(steering_coeff);
  controller.SetThrottleCoeff(throttle_coeff);
  
  bool display_rviz = display == "rviz";
  auto next_waypoint_pub = display_rviz ? n->create_publisher<avt_341::msg::PointStamped>("avt_341/control_next_waypoint", 1) : nullptr;

  controller.SetWheelbase(wheelbase);
	controller.SetMaxSteering(steer_angle*3.14159 / 180.0);
	controller.SetDesiredSpeed(vehicle_speed);
  controller.SetSpeedControllerParams(throttle_kp, throttle_ki, throttle_kd);

  float rate = 100.0f;
  float dt = 1.0f/rate;
  float brake_step = dt/time_to_max_brake;
  float current_brake_value = 0.0;

  avt_341::node::Rate r(rate);
  avt_341::utils::vec2 goal;

  while (avt_341::node::ok()){
    avt_341::msg::Twist dc;
    bool time_to_quit = false;

    // tell the controller the current vehicle state
    controller.SetVehicleState(state);

    if (current_run_state==0){    // active running state
      controller.SetDesiredSpeed(vehicle_speed);
      dc = controller.GetDcFromTraj(control_msg, goal);
    }
    else if (current_run_state==-1 || current_run_state==1){
      // bring to a smooth stop and wait / idle
      controller.SetDesiredSpeed(0.0f);
      dc = controller.GetDcFromTraj(control_msg, goal);
    }
    else if (current_run_state==2){ 
      // bring to a smooth stop and shut down
      controller.SetDesiredSpeed(0.0f);
      float vel = sqrtf(state.twist.twist.linear.x*state.twist.twist.linear.x + state.twist.twist.linear.y*state.twist.twist.linear.y);
      if (vel<0.5f)time_to_quit = true;
      dc = controller.GetDcFromTraj(control_msg, goal);
      dc.linear.x = 0.0f;
      dc.angular.z = 0.0f;

    }
    else if (current_run_state==3){
      // bring to a hard stop and shut down
      dc.linear.x = 0.0f;
      dc.linear.y = 1.0f;
      dc.angular.z = 0.0f;
      time_to_quit = true;
    }
    
    // check braking and throttle
    if (dc.linear.y!=0.0){
      // apply the ramp up to the brake
      if (current_brake_value>dc.linear.y){
        dc.linear.y = current_brake_value - brake_step;
        if (dc.linear.y<-1.0)dc.linear.y = -1.0;
        if (dc.linear.y>0.0)dc.linear.y = 0.0;
      }
      // make sure the throttle is zero when braking
      dc.linear.x = 0.0f;
    }
    // publish the driving command
    dc_pub->publish(dc);
    current_brake_value = dc.linear.y;

    // break the loop when an end state is reached
    if (time_to_quit)break;
    
    if(display_rviz){
      avt_341::msg::PointStamped next_waypoint_msg;
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
