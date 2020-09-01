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

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
//avt_341 includes
#include "avt_341/control/pure_pursuit_controller.h"

nav_msgs::Path control_msg;
nav_msgs::Odometry state;

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& rcv_state) {
	state = *rcv_state; 
}

void PathCallback(const nav_msgs::Path::ConstPtr& rcv_control){
  control_msg.poses = rcv_control->poses;
  control_msg.header = rcv_control->header;
}

int main(int argc, char *argv[]){
  ros::init(argc,argv,"avt_341_control_node");
  ros::NodeHandle n;

  ros::Publisher dc_pub =
    n.advertise<geometry_msgs::Twist>("avt_341/cmd_vel",1);

  ros::Subscriber path_sub =
    n.subscribe("avt_341/local_path",1, PathCallback);

  ros::Subscriber state_sub =
    n.subscribe("avt_341/odometry",1, OdometryCallback);

  avt_341::control::PurePursuitController controller;
	// Set controller parameters
	float wheelbase = 2.6f;
	if (ros::param::has("~vehicle_wheelbase")) {
		ros::param::get("~vehicle_wheelbase", wheelbase);
	}
	float steer_angle = 25.0f;
	if (ros::param::has("~vehicle_max_steer_angle_degrees")) {
		ros::param::get("~vehicle_max_steer_angle_degrees", steer_angle);
	}
	float vehicle_speed = 5.0f;
	if (ros::param::has("~vehicle_speed")) {
		ros::param::get("~vehicle_speed", vehicle_speed);
	}
  if (ros::param::has("~steering_coefficient")) {
    float steering_coeff;
		ros::param::get("~steering_coefficient", steering_coeff);
    controller.SetSteeringParam(steering_coeff);
	}
  
	controller.SetWheelbase(wheelbase);
	controller.SetMaxSteering(steer_angle*3.14159 / 180.0);
	controller.SetDesiredSpeed(vehicle_speed);

  float rate = 50.0f;
  float dt = 1.0f/rate;
  ros::Rate r(rate);

  while (ros::ok()){
    geometry_msgs::Twist dc;

    controller.SetVehicleState(state);
    dc = controller.GetDcFromTraj(control_msg);

    
    dc_pub.publish(dc);
    ros::spinOnce();

    r.sleep();
  }

  return 0;
}
