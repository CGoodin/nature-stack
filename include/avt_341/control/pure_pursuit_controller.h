/**
* \class PurePursuitController
*
* A pure-pursuit vehicle control that follows an input vehicle
* trajectory in 2D space. 
* 
* See "Implementation of the Pure Pursuit Path Tracking Algorithm"
* by Craig Coulter, CMU-RI-TR-92-01
* 
* and
* 
* "Automatic Steering Methods for Autonomous Automobile Path Tracking"
* by Jarrod M. Snider, CMU-RI-TR-09-08
*
* \author Chris Goodin
*
* \date 8/31/2020
*/
#ifndef PURE_PURSUIT_CONTROLLER_H
#define PURE_PURSUIT_CONTROLLER_H

#include "avt_341/control/pid_controller.h"
#include "avt_341/node/ros_types.h"
#include "avt_341/avt_341_utils.h"

namespace avt_341 {
namespace control{

class PurePursuitController {
public:
	/// Create a controller
	PurePursuitController();

	/**
	* Calculate a driving command based on a trajectory
	* The first point on the trajectory must be the current
	* vehicle state.
	* \param traj The desired trajectory
	*/
	avt_341::msg::Twist GetDcFromTraj(avt_341::msg::Path traj, utils::vec2 & goal);

	/**
	* Set the wheelbase of the vehicle in meters
	* \param wb Wheelbase to set
	*/
	void SetWheelbase(float wb) { wheelbase_ = wb; }

	/**
	* Set the max steering angle of the vehicle in radians
	* \param st Max steering angle
	*/
	void SetMaxSteering(float st) { max_steering_angle_ = st; }

	/** 
	* Set the minimum look-ahead distance of the planner, in meters
	* \param min_la The minimum look-ahead distance
	*/
	void SetMinLookAhead(float min_la) { min_lookahead_ = min_la; }

	/**
	* Set the maximum look-ahead distance of the planner, in meters
	* \param max_la The maximum look-ahead distance
	*/
	void SetMaxLookAhead(float max_la) { max_lookahead_ = max_la; }

	/** 
	* Set the gain factor on the steering controller
	* \param k Desired gain factor
	*/
	void SetSteeringParam(float k) { k_ = k; }

	/**
	* Set the maximum allowed speed of the vehicle
	* Contoller will limit throttle to stay below this speed
	* \param speed Maximum desired speed in m/s
	*/
	void SetMaxStableSpeed(float speed) { max_stable_speed_ = speed; }

	/**
	* Set the desired speed of the vehicle in m/s
	* \param speed The desired speed
	*/
	void SetDesiredSpeed(float speed) {
		desired_speed_ = speed;
		speed_controller_.SetSetpoint(speed);
	}

	/**
	* Set the coefficients of the PID speed controller
	* \param kp Proportional coefficient
	* \param ki Integral coefficient
	* \param kd Derivative coefficient
	*/
	void SetSpeedControllerParams(float kp, float ki, float kd) {
		speed_controller_.SetKp(kp);
		speed_controller_.SetKi(ki);
		speed_controller_.SetKd(kd);
	}

	/**
	* Set the current vehicle position in local ENU
	* \param x Current x-coordinate in ENU
	* \param y Current y-coordinate in ENU
	*/
	void SetVehiclePosition(float x, float y) {
		veh_x_ = x;
		veh_y_ = y;
	}

	/**
	* Set the current vehicle speed in m/s
	* \param speed The current vehicle speed 
	*/
	void SetVehicleSpeed(float speed) {
		veh_speed_ = speed;
	}

	/**
	* Set the current vehicle heading in radians
	* \param heading The current vehicle heading
	*/
	void SetVehicleOrientation(float heading) {
		veh_heading_ = heading;
	}

	/**
	 *  Set the vehicle position, orientation and speed
	 * \param state The vehicle state
	 */
	void SetVehicleState(avt_341::msg::Odometry state);

private:
	float wheelbase_; //meters
	float max_steering_angle_; //radians
	float min_lookahead_; //meters
	float max_lookahead_; //meters
	float k_; //unitless
	float desired_speed_; // m/s
	float max_stable_speed_;
	PidController speed_controller_;

	//current vehicle state info
	float veh_x_;
	float veh_y_;
	float veh_heading_;
	float veh_speed_;
	float vx_;
	float vy_;
};

} // namespace control
} // namespace avt_341

#endif
