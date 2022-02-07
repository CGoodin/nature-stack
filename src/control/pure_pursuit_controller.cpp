#include "avt_341/control/pure_pursuit_controller.h"

namespace avt_341 {
namespace control{

PurePursuitController::PurePursuitController() {
	skid_steered_ = false;

	//default wheelbase and steer angle
	// set to MRZR values
	wheelbase_ = 2.731f; // meters
	max_steering_angle_ = 0.69f; //39.5 degrees
	max_stable_speed_ = 35.0f; //5.0;

	// tunable parameters
	min_lookahead_ = 2.0f;
	max_lookahead_ = 25.0f;
	k_ = 2.0f; //0.5;
	throttle_coeff_ = 1.0f;

	//vehicle state parameters
	veh_x_ = 0.0f;
	veh_y_ = 0.0f;
	veh_speed_ = 0.0f;
	vx_ = 0.0f;
	vy_ = 0.0f;

	k_theta_ = 1.0f;
	kx_ = 1.0f;
	ky_ = 1.0f;
}

void PurePursuitController::SetVehicleState(avt_341::msg::Odometry state){
// Set the current state of the vehicle, which should be the first pose in the path
	veh_x_ = state.pose.pose.position.x;
	veh_y_ = state.pose.pose.position.y;
	vx_ = state.twist.twist.linear.x;
	vy_ = state.twist.twist.linear.y;
	current_angular_velocity_ = state.twist.twist.angular.z;
	veh_speed_ = sqrt(vx_*vx_ + vy_*vy_);
	veh_heading_ = utils::GetHeadingFromOrientation(state.pose.pose.orientation);
}

void PurePursuitController::SetVehicleSpeed(float speed){
	veh_speed_ = speed;
	vx_ = cosf(veh_heading_)*veh_speed_;
	vy_ = sinf(veh_heading_)*veh_speed_;
}

avt_341::msg::Twist PurePursuitController::GetDcFromTraj(avt_341::msg::Path traj, utils::vec2 & goal) {
	//initialize the driving command
  avt_341::msg::Twist dc;

	//make sure the path contains some points
	int np = traj.poses.size();

	if (np < 2) return dc;

	// extract the path that the vehicle needs to follow
	std::vector<utils::vec2> path;

	//populate the desired path
	path.resize(np);
	for (int i = 0; i < np; i++) {
		path[i] = utils::vec2(traj.poses[i].pose.position.x, traj.poses[i].pose.position.y);
	}

	//calculate the lookahead distance based on current speed
	utils::vec2 currpos(veh_x_, veh_y_);
	float path_length = utils::length(path[np - 1] - currpos);
	float lookahead = k_ * veh_speed_;

	if (lookahead > max_lookahead_)lookahead = max_lookahead_;
	if (lookahead < min_lookahead_)lookahead = min_lookahead_;
	if (lookahead > path_length)lookahead = path_length - 0.01;

	//first find the closest segment on the path , and distance to it
	float closest = 1.0E9f;
	int start_seg = 0;
	for (int i = 0; i < np - 1; i++) {
		float d0 = PointToSegmentDistance(path[i], path[i + 1], currpos);
		if (d0 < closest) {
			closest = d0;
			start_seg = i;
		}
	}

	goal = path[start_seg];
	float target_speed = desired_speed_;
	utils::vec2 desired_direction;
	if (closest < lookahead) {
		//find point on path at lookahead distance away
		float accum_dist = closest;

		//for (int i=0;i<np-1;i++){
		for (int i = start_seg; i < np - 1; i++) {
			utils::vec2 v = path[i + 1] - path[i];
			float seg_dist = length(v);
			if ((accum_dist + seg_dist) > lookahead) {
				utils::vec2 dir = v / seg_dist;
				float t = lookahead - accum_dist;
				goal = path[i] + dir*t;
				desired_direction = v;
				target_speed = desired_speed_; //traj.path[i + 1].speed;
				if (target_speed > max_stable_speed_)target_speed = max_stable_speed_;
				break;
			}
			else {
				accum_dist += seg_dist;
			}
		}
	}

	//find the angle, alpha, between the current orientation and the goal
	utils::vec2 curr_dir(cos(veh_heading_), sin(veh_heading_));
	utils::vec2 to_goal(goal.x - veh_x_,goal.y-veh_y_);
	//to_goal = to_goal / utils::length(to_goal);
	float alpha = (float)atan2(to_goal.y, to_goal.x) - (float)atan2(curr_dir.y, curr_dir.x);

	if (skid_steered_){
		float desired_heading = atan2f(desired_direction.y, desired_direction.x);
		float dtheta = desired_heading - veh_heading_;
		dc = GetDcSkid(to_goal.x, to_goal.y, dtheta);
	}
	else{
		dc = GetDcAckermann(alpha, lookahead, curr_dir, target_speed);
	}
	return dc;
}

avt_341::msg::Twist PurePursuitController::GetDcSkid(float dx, float dy, float dtheta){
	// The skid steer algorithm is taken from 
	// A Stable Tracking Control Method for a Non-Holonomic Mobile Robot
	// Yutaka Kanayam, 1991
	// Proceedings of IROS 91
	// NOTE: The output is the typical cmd_vel message for a mobile robot where
	// the velocities are true velocities, not throttle-steering-brake commands like 
	// the controller for the Ackerman vehicle
	avt_341::msg::Twist dc;
	dc.linear.x = 0.0;
	dc.linear.y = 0.0;
	dc.linear.z = 0.0;
	dc.angular.x = 0.0;
	dc.angular.y = 0.0;
	dc.angular.z = 0.0;

	float vr = desired_speed_;
	float ct = cosf(veh_heading_);
	float st = sinf(veh_heading_);
	float xe = ct*dx + st*dy;
	float ye = -st*dx + ct*dy;

	float v = vr*cosf(dtheta) + kx_*xe;
	float w = current_angular_velocity_ + vr*(ky_*ye + k_theta_*sinf(dtheta));

	dc.linear.x = v*ct;
	dc.linear.y = v*st;
	dc.angular.z = w;

	return dc;
}

avt_341::msg::Twist PurePursuitController::GetDcAckermann(float alpha, float lookahead, utils::vec2 curr_dir, float target_speed){
	avt_341::msg::Twist dc;
	dc.linear.x = 0.0;
	dc.angular.z = 0.0;
	dc.linear.y = 0.0;

	//determine the desired normalized steering angle
	float sangle = (float)atan2(2 * wheelbase_*sin(alpha), lookahead);
	sangle = sangle / max_steering_angle_;
	sangle = std::min(1.0f, sangle);
	sangle = std::max(-1.0f, sangle);
	dc.angular.z = sangle;

	//Use the speed controller to get throttle/braking
	//addjust the target speed so you back off during hard turns
	//float adj_speed = target_speed * exp(-0.69*pow(fabs(dc.angular.z), 4.0f));
	//speed_controller_.SetSetpoint(adj_speed);
	speed_controller_.SetSetpoint(target_speed);
	float vdot = vx_*curr_dir.x + vy_*curr_dir.y;
	float throttle = speed_controller_.GetControlVariable(veh_speed_, 0.01f);
	//float throttle = speed_controller_.GetControlVariable(vdot, 0.01f);
	if (throttle < 0.0f) { //braking
		dc.linear.x = 0.0f;
		dc.linear.y = 0.0; //std::max(-1.0f, throttle);
	}
	else {
		dc.linear.y = 0.0f;
		dc.linear.x = std::min(1.0f, throttle);
	}

	dc.linear.x = throttle_coeff_*dc.linear.x;

	return dc;
} // GetDcAcerman

} // namespace control
} // namespace avt_341
