/**
 * \class DwaPlanner
 *
 * Class for the Dynamic Window Approach planner.
 *
 * author: Atsushi Sakai (@Atsushi_twi)
 * Ref:
 * https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
 *
 * Original source code from:
 * https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/PotentialFieldPlanning\
 *
 * Converted to C++ by Copilot
 * Modified by CTG to be faster, integrate with ROS, and work with an Occupancy grid.
 *
 * \author Chris Goodin
 *
 * \date 10/13/2025
 */
// c++ includes
#include <array>
#include <vector>
// ROS includes
#include "nature/node/ros_types.h"
#include "nature/node/node_proxy.h"

namespace nature {
namespace planning {

enum class RobotType { Circle = 0, Rectangle = 1 };

/// Vehicle state:
struct State {
	double x{ 0.0 }; // meters
	double y{ 0.0 }; // meters
	double yaw{ 0.0 }; // radians
	double v{ 0.0 }; // m/s
	double omega{ 0.0 }; // rad/s
	double steer{ 0.0 }; // rad
};

/// Robot Config
struct Config {
	// --- Robot & DWA parameters (shared) ---
	double max_speed = 1.0;                 // [m/s]
	double min_speed = -0.5;                // [m/s] (set to 0.0 to forbid reversing)
	double max_accel = 0.2;                 // [m/ss]
	double v_resolution = 0.01;             // [m/s]
	double dt = 0.1;                        // [s]  time tick for motion prediction
	double predict_time = 3.0;              // [s]  rollout horizon

	// --- Ackermann / Bicycle parameters ---
	double wheelbase = 2.5;                 // [m] wheelbase L
	double steer_max = 0.6;                 // [rad] ~34 deg
	double steer_rate_max = 0.5;            // [rad/s]
	double steer_resolution = 0.01;         // [rad]

	// --- Cost gains ---
	double to_goal_cost_gain = 0.15;
	double speed_cost_gain = 1.0;
	double obstacle_cost_gain = 1.0;

	double robot_stuck_flag_cons = 0.001;   // stuck prevention threshold

	RobotType robot_type = RobotType::Rectangle; // default same as original __main__

	// If Circle (also used as goal radius for both types)
	double robot_radius = 1.0;              // [m]

	// If Rectangle (footprint)
	double robot_width = 0.5;              // [m]
	double robot_length = 1.2;              // [m]
};

// ----- Dynamic window in (v, steer) space -----
struct WindowVS { double vmin, vmax, smin, smax; };

class DwaPlanner {
public:
	DwaPlanner();

	/**
	* Set the goal point in the local ENU coordinate frame
	*/
	void SetGoal(float gx, float gy) {
		goal_[0] = gx;
		goal_[1] = gy;
	}

	nature::msg::Path Plan(nature::msg::OccupancyGrid *grid, nature::msg::Odometry odom, double current_steering);

private:

	void PopulateObstacles(nature::msg::OccupancyGrid* grid);

	// ----- Bicycle model motion (applies accel & steer rate limits per step) -----
	State MotionBicycle(State tmp_state, double v_cmd, double steer_cmd);

	// ----- Predict trajectory with constant (v, steer) command over horizon -----
	std::vector<State> PredictTrajBicycle(double v_cmd, double steer_cmd);

	WindowVS CalcWindowVs();

	// ----- Costs -----
	double CalcToGoalCost(std::vector<State> test_traj);

	// Obstacle cost: +inf if collision; else 1/min_distance (same as original)
	double CalcObstacleCost(std::vector<State> test_traj);
	
	// ----- Core DWA search in (v, steer) -----
	std::pair<std::array<double, 2>, std::vector<State>> CalcControlAndTrajectoryAck(const WindowVS& win);
	
	std::pair<std::array<double, 2>, std::vector<State>> DwaControlAck();

	State state_;
	Config config_;
	std::array<double, 2> goal_;
	std::vector<std::array<double, 2>> obstacles_;

}; // class DwaPlanner

} // namespace planning
} // namespace nature