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
	float wheelbase, steer_angle, vehicle_speed, steering_coeff;
	n->get_parameter("~vehicle_wheelbase", wheelbase, 2.6f);
    n->get_parameter("~vehicle_max_steer_angle_degrees", steer_angle, 25.0f);
    n->get_parameter("~vehicle_speed", vehicle_speed, 5.0f);
    n->get_parameter("~steering_coefficient", steering_coeff, 2.0f);
    controller.SetSteeringParam(steering_coeff);

  
	controller.SetWheelbase(wheelbase);
	controller.SetMaxSteering(steer_angle*3.14159 / 180.0);
	controller.SetDesiredSpeed(vehicle_speed);

  float rate = 50.0f;
  float dt = 1.0f/rate;
  avt_341::node::Rate r(rate);

  while (avt_341::node::ok()){
    avt_341::msg::Twist dc;

    controller.SetVehicleState(state);
    if (current_run_state==0){    // active running state
      controller.SetDesiredSpeed(vehicle_speed);
      //controller.SetVehicleState(state);
      dc = controller.GetDcFromTraj(control_msg);
      dc_pub->publish(dc);
    }
    else if (current_run_state==-1 || current_run_state==1){    // startup or smooth stop
      // bring to a smooth stop and wait
      controller.SetDesiredSpeed(0.0f);
      //controller.SetVehicleState(state);
      dc = controller.GetDcFromTraj(control_msg);
      dc_pub->publish(dc);
    }
    else if (current_run_state==2){ 
      // bring to a smooth stop and shut down
      controller.SetDesiredSpeed(0.0f);
      float vel = sqrtf(state.twist.twist.linear.x*state.twist.twist.linear.x + state.twist.twist.linear.y*state.twist.twist.linear.y);
      if (vel<0.5f){
        // bring to a stop and shut down
        dc.linear.x = 0.0f;
        dc.linear.y = 1.0f;
        dc.angular.z = 0.0f;
        dc_pub->publish(dc);
        break;   
      }
      else{
       // controller.SetVehicleState(state);
        dc = controller.GetDcFromTraj(control_msg);
        dc.linear.x = 0.0f;
        dc.angular.z = 0.0f;
        dc_pub->publish(dc);
      }
    }
    else if (current_run_state==3){
      // bring to a hard stop and shut down
      dc.linear.x = 0.0f;
      dc.linear.y = 1.0f;
      dc.angular.z = 0.0f;
      dc_pub->publish(dc);
      break;
    }
    
    n->spin_some();

    r.sleep();
  }

  return 0;
}
