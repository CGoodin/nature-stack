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
#include <iostream>
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
//avt_341 includes
#include "avt_341/control/pure_pursuit_controller.h"
#include "avt_341/control/tinyfiledialogs.h"

avt_341::msg::Path control_msg;
avt_341::msg::Odometry state;
int current_run_state = -1;   // startup state
bool shutdown_condition = false;
double mrzr_speedometer = 0.0;
bool speedometer_rcvd = false;
double mrzr_steering = 0.0;
bool path_rcvd = false;

void OdometryCallback(avt_341::msg::OdometryPtr rcv_state) {
	state = *rcv_state; 
}

void SpeedCallback(avt_341::msg::Float64Ptr rcv_speed) {
	mrzr_speedometer = rcv_speed->data;
  speedometer_rcvd = true; 
}

void SteeringCallback(avt_341::msg::Float64Ptr rcv_steering) {
	mrzr_steering = rcv_steering->data;
}

void PathCallback(avt_341::msg::PathPtr rcv_control){
  control_msg.poses = rcv_control->poses;
  control_msg.header = rcv_control->header;
  path_rcvd = true;
}

void StateCallback(avt_341::msg::Int32Ptr rcv_state){
  current_run_state = rcv_state->data;
  if (current_run_state==2)shutdown_condition = true;
}

double length(avt_341::msg::Point a, avt_341::msg::Point b){
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  double dz = a.z - b.z;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

float TriangleArea(avt_341::msg::Point a, avt_341::msg::Point b, avt_341::msg::Point c) {
	float area = (float)fabs(a.x*(b.y - c.y) + b.x*(c.y - a.y) + c.x*(a.y - b.y));
	return area;
}

float MengerCurvature(avt_341::msg::Point a, avt_341::msg::Point b, avt_341::msg::Point c) {
	float curv = 0.0f;
	float denom = length(a, b)*length(b, c)*length(c, b);
	if (denom == 0.0f) {
		curv = std::numeric_limits<float>::max();
	}
	else {
		float area = TriangleArea(a, b, c);
		curv = 4.0f*area / denom;
	}
	return curv;
}

double GetMaxCurvature(avt_341::msg::Path path){
  double max_curvature = 0.0;
  if (path.poses.size() > 2) {
		for (int i = 1; i < path.poses.size() - 1; i++){
		  double curvature = MengerCurvature(path.poses[i - 1].pose.position, path.poses[i].pose.position, path.poses[i + 1].pose.position);
      if (curvature>max_curvature)max_curvature = curvature;
		}
	}
  return max_curvature;
}


int main(int argc, char *argv[]){
  auto n = avt_341::node::init_node(argc,argv,"avt_341_control_node");

  auto dc_pub = n->create_publisher<avt_341::msg::Twist>("avt_341/cmd_vel",1);

  auto path_sub = n->create_subscription<avt_341::msg::Path>("avt_341/local_path",1, PathCallback);

  auto state_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry",1, OdometryCallback);

  auto control_sub = n->create_subscription<avt_341::msg::Int32>("avt_341/state",1,StateCallback);

  auto speed_sub = n->create_subscription<avt_341::msg::Float64>("mrzr_velocity",1,SpeedCallback);

  auto steering_sub = n->create_subscription<avt_341::msg::Float64>("mrzr_steering",1,SteeringCallback);


  avt_341::control::PurePursuitController controller;

  // added by CTG, 1/26/22
  // The PID params are tuned with this value in mind
  // so it's not a good idea to change it
  float time_to_max_throttle = 3.0f; //seconds
	// Set controller parameters
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

  // Get the parameters for a skid steered vehicle
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

  if (use_feed_forward){
    controller.GetPidSpeedController()->SetUseFeedForward(true);
    controller.GetPidSpeedController()->SetForwardModelParams(ff_a0, ff_a1, ff_a2);
  }
  
  controller.SetDesiredSpeed(vehicle_speed);
  if (turn_off_velocity_overshoot_corrector){
    controller.GetPidSpeedController()->SetOvershootLimiter(false);
  }

  bool display_rviz = display == "rviz";
  auto next_waypoint_pub = display_rviz ? n->create_publisher<avt_341::msg::PointStamped>("avt_341/control_next_waypoint", 1) : nullptr;

  float rate = 100.0f;
  float dt = 1.0f/rate;
  float brake_step = dt/time_to_max_brake;
  float max_throttle_step = dt/time_to_max_throttle;
  float max_steering_step = dt/time_to_max_steering;
  float current_brake_value = 0.0f;
  float current_throttle_value = 0.0f;
  float current_steering_value = 0.0f;
  bool user_approved = false;
  avt_341::node::Rate r(rate);
  avt_341::utils::vec2 goal;

  while (avt_341::node::ok()){
    avt_341::msg::Twist dc;
    bool time_to_quit = false;

    // tell the controller the current vehicle state
    float vel = 0.0f;
    if (speedometer_rcvd){
      vel = mrzr_speedometer;
      current_steering_value = mrzr_steering;
    }
    else{
      vel = sqrtf(state.twist.twist.linear.x*state.twist.twist.linear.x + state.twist.twist.linear.y*state.twist.twist.linear.y);
    }
    
    controller.SetVehicleState(state);
    controller.SetVehicleSpeed(vel);

    if (shutdown_condition){  // current_run_state = 2 
      // bring to a smooth stop and shut down
      controller.SetDesiredSpeed(0.0f);
      if (vel<0.5f)time_to_quit = true;
      dc = controller.GetDcFromTraj(control_msg, goal);
      dc.linear.x = 0.0f;
      dc.angular.z = 0.0f;
      dc.linear.y = -1.0f;
    }
    else if (current_run_state==0){    // active running state
      double max_curvature = GetMaxCurvature(control_msg);
      double lateral_g_force = ((vel*vel)*max_curvature)/9.806;
      float desired_velocity = vehicle_speed;
      if (lateral_g_force>max_desired_lateral_g){
        desired_velocity = sqrt(9.806*max_desired_lateral_g/max_curvature);
        if (desired_velocity>vehicle_speed)desired_velocity=vehicle_speed;
      }
      controller.SetDesiredSpeed(desired_velocity);
      //controller.SetDesiredSpeed(vehicle_speed);
      dc = controller.GetDcFromTraj(control_msg, goal);
    }
    else if (current_run_state==-1 || current_run_state==1){
      // bring to a smooth stop and wait / idle
      controller.SetDesiredSpeed(0.0f);
      dc = controller.GetDcFromTraj(control_msg, goal);
      if (current_run_state==-1)dc.linear.x = 0.0f;
    }
    else if (current_run_state==3){
      // bring to a hard stop and shut down
      dc.linear.x = 0.0f;
      dc.linear.y = 1.0f;
      dc.angular.z = 0.0f;
      time_to_quit = true;
    }

    if (!skid_steered){
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
      // apply the throttle ramp up
      if (dc.linear.x-current_throttle_value > max_throttle_step){
        dc.linear.x = current_throttle_value + max_throttle_step;
      }
      // apply the steering ramp up
      //if (fabs(dc.angular.z-current_steering_value)>max_steering_step){
      //  dc.angular.z = current_steering_value + max_steering_step*(dc.angular.z-current_steering_value)/fabs(dc.angular.z-current_steering_value);
      //}
    }
    // publish the driving command
    dc_pub->publish(dc);
    current_brake_value = dc.linear.y;
    current_throttle_value = dc.linear.x;
    current_steering_value = dc.angular.z; 


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

    // ask the user if the path looks good and they would like to continue
    if (!user_approved && current_run_state==0 && path_rcvd && request_approval){
      std::string message_string("Do you approve the initial conditions? \n Click Yes to continue experiment.");
		  bool approved = tinyfd_messageBox("Approve initial conditions", message_string.c_str(), "yesno", "question", 1);
      if (approved){
        user_approved = true;
      }
      else{
        break;
      }
    }


    n->spin_some();

    r.sleep();
  }

  return 0;
}
