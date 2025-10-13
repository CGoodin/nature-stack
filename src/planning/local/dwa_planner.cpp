// dynamic_window_approach_ackermann.cpp
// DWA for Ackermann (bicycle) vehicles — converted & adapted by M365 Copilot for Chris Goodin.
//
// Key changes vs. your unicycle version:
//  - Samples controls in (v, steer) space and enforces |dv/dt| <= a_max and |d(steer)/dt| <= steer_rate_max.
//  - Trajectory rollouts use the bicycle model: yaw_dot = v/L * tan(steer).
//
// Build:  g++ -O2 -std=c++17 dynamic_window_approach_ackermann.cpp -o dwa_ack
// Run:    ./dwa_ack
//

#include "nature/planning/local/dwa_planner.h"
#include "nature/nature_utils.h"
#include <limits>

namespace nature {
namespace planning {


static inline double WrapToPi(double ang) {
	return std::atan2(std::sin(ang), std::cos(ang));
}

DwaPlanner::DwaPlanner() {
	// Notes:
	//  - Set wheelbase, steering limits, and rates in Config below.
	//  - If you want to disable reversing, set min_speed = 0.0.
	// Adjust for your platform:
	config_.robot_type = RobotType::Rectangle; // mirrors original default
	config_.wheelbase = 2.5;                  // [m]
	config_.steer_max = 0.6;                  // [rad]
	config_.steer_rate_max = 0.5;                  // [rad/s]
	config_.min_speed = -0.5;                 // allow reverse; set 0.0 to forbid
	config_.max_speed = 1.0;                  // [m/s]

	// Initial state
	state_.x = 0.0; state_.y = 0.0; state_.yaw = 0.0; state_.v = 0.0; state_.omega = 0.0; state_.steer = 0.0;

	goal_[0] = 10.0;
	goal_[1] = 10.0;
}

// ----- Bicycle model motion (applies accel & steer rate limits per step) -----
State DwaPlanner::MotionBicycle(State in_state, double v_cmd, double steer_cmd) {
	State ns = in_state;

	// Apply first-order acceleration and steering-rate limiting
	const double dv = std::clamp(v_cmd - in_state.v, -config_.max_accel * config_.dt, config_.max_accel * config_.dt);
	const double dsteer = std::clamp(steer_cmd - in_state.steer, -config_.steer_rate_max * config_.dt, config_.steer_rate_max * config_.dt);

	ns.v = std::clamp(in_state.v + dv, config_.min_speed, config_.max_speed);
	ns.steer = std::clamp(in_state.steer + dsteer, -config_.steer_max, config_.steer_max);

	// Bicycle kinematics
	const double tan_steer = std::tan(ns.steer);
	ns.omega = (std::abs(std::cos(ns.steer)) < 1e-6) ? 0.0 : (ns.v / config_.wheelbase) * tan_steer;

	ns.yaw += ns.omega * config_.dt;
	ns.x += ns.v * std::cos(ns.yaw) * config_.dt;
	ns.y += ns.v * std::sin(ns.yaw) * config_.dt;

	return ns;
}

// ----- Predict trajectory with constant (v, steer) command over horizon -----
std::vector<State> DwaPlanner::PredictTrajBicycle(double v_cmd, double steer_cmd) {
	std::vector<State> traj;
	traj.reserve(static_cast<std::size_t>(config_.predict_time / config_.dt) + 2);
	State x = state_;
	traj.push_back(x);
	for (double t = 0.0; t <= config_.predict_time + 1e-12; t += config_.dt) {
		x = MotionBicycle(x, v_cmd, steer_cmd);
		traj.push_back(x);
	}
	return traj;
}

WindowVS DwaPlanner::CalcWindowVs() {
	// Speeds reachable in one dt (accel bound) intersected with global min/max
	const double vmin = std::max(config_.min_speed, state_.v - config_.max_accel * config_.dt);
	const double vmax = std::min(config_.max_speed, state_.v + config_.max_accel * config_.dt);

	// Steering reachable in one dt (steer-rate bound) intersected with global +/- steer_max
	const double smin = std::max(-config_.steer_max, state_.steer - config_.steer_rate_max * config_.dt);
	const double smax = std::min(config_.steer_max, state_.steer + config_.steer_rate_max * config_.dt);

	return { vmin, vmax, smin, smax };
}

// ----- Costs -----
double DwaPlanner::CalcToGoalCost(std::vector<State> traj) {
	const State& last = traj.back();
	const double dx = goal_[0] - last.x;
	const double dy = goal_[1] - last.y;
	const double error_angle = std::atan2(dy, dx);
	const double cost_angle = error_angle - last.yaw;
	return std::abs(WrapToPi(cost_angle));
}

// Obstacle cost: +∞ if collision; else 1/min_distance (same as original)
double DwaPlanner::CalcObstacleCost(std::vector<State> traj)
{
	const double INF = std::numeric_limits<double>::infinity();

	if (config_.robot_type == RobotType::Circle) {
		double min_r = INF;
		for (const auto& s : traj) {
			for (const auto& o : obstacles_) {
				const double r = std::hypot(s.x - o[0], s.y - o[1]);
				if (r <= config_.robot_radius) return INF; // collision
				if (r < min_r) min_r = r;
			}
		}
		if (!std::isfinite(min_r) || min_r <= 1e-12) return INF;
		return 1.0 / min_r;
	}

	// Rectangle footprint
	if (config_.robot_type == RobotType::Rectangle) {
		const double hx = config_.robot_length / 2.0;
		const double hy = config_.robot_width / 2.0;

		double min_r = INF;
		for (const auto& s : traj) {
			const double cy = std::cos(s.yaw);
			const double sy = std::sin(s.yaw);

			// Transform obstacles into robot frame using rot = [[cos, -sin],[sin, cos]]
			for (const auto& o : obstacles_) {
				const double rx = o[0] - s.x;
				const double ry = o[1] - s.y;
				const double lx = rx * cy + ry * sy;
				const double ly = rx * (-sy) + ry * cy;

				if ((lx <= hx) && (lx >= -hx) && (ly <= hy) && (ly >= -hy)) {
					return INF; // collision
				}
				const double r = std::hypot(rx, ry);
				if (r < min_r) min_r = r;
			}
		}
		if (!std::isfinite(min_r) || min_r <= 1e-12) return INF;
		return 1.0 / min_r;
	}

	return std::numeric_limits<double>::infinity();
}

// ----- Core DWA search in (v, steer) -----
std::pair<std::array<double, 2>, std::vector<State>> DwaPlanner::CalcControlAndTrajectoryAck(const WindowVS& win){
	const double INF = std::numeric_limits<double>::infinity();

	double min_cost = INF;
	std::array<double, 2> best_u{ 0.0, 0.0 }; // [v_cmd, steer_cmd]
	std::vector<State> best_traj{ state_ };

	for (double v = win.vmin; v <= win.vmax + 1e-12; v += config_.v_resolution) {
		for (double steer = win.smin; steer <= win.smax + 1e-12; steer += config_.steer_resolution) {

			// Optional: enforce curvature at high speeds? (already implicit via steer bounds)
			auto traj = PredictTrajBicycle(v, steer);

			const double to_goal_cost = config_.to_goal_cost_gain * CalcToGoalCost(traj);
			const double speed_cost = config_.speed_cost_gain * (config_.max_speed - std::abs(traj.back().v));
			const double ob_cost = config_.obstacle_cost_gain * CalcObstacleCost(traj);

			const double final_cost = to_goal_cost + speed_cost + ob_cost;

			if (final_cost <= min_cost) {
				min_cost = final_cost;
				best_u = { v, steer };
				best_traj = std::move(traj);
			}
		}
	}

	// Stuck prevention: if nearly zero speed selected and currently nearly stopped, bias a turn
	if (std::abs(best_u[0]) < config_.robot_stuck_flag_cons &&
		std::abs(state_.v) < config_.robot_stuck_flag_cons)
	{
		// Nudge steering to break symmetry (half max to avoid instant saturation)
		best_u[1] = -0.5 * config_.steer_max;
	}

	return { best_u, best_traj };
}

std::pair<std::array<double, 2>, std::vector<State>> DwaPlanner::DwaControlAck(){
	const WindowVS w = CalcWindowVs();
	return CalcControlAndTrajectoryAck(w);
}

void DwaPlanner::PopulateObstacles(nature::msg::OccupancyGrid *grid) {
	float sx = state_.x;
	float sy = state_.y;
	float gx = goal_[0];
	float gy = goal_[1];
	obstacles_.clear();
	int ndx = 0;
	for (int i = 0; i < grid->info.width; i++) {
		float x = grid->info.origin.position.x + (i + 0.5f) * grid->info.resolution;
		float dx = sx - x;
		for (int j = 0; j < grid->info.height; j++) {
			float y = grid->info.origin.position.y + (j + 0.5f) * grid->info.resolution;
			float dy = sy - y;
				if (grid->data[ndx] > 0.0) {
				float d = sqrtf(dx * dx + dy * dy);
				//if (d < obs_cutoff_dist_ && d>inner_cutoff_dist_) {
					std::array<double, 2> ob;
					ob[0] = x;
					ob[1] = y;
					obstacles_.push_back(ob);
				//} // if closer than cutoff distance
			} // if cell occupied
			ndx++;
		} // loop over j
	} // loop over i
}
nature::msg::Path DwaPlanner::Plan(nature::msg::OccupancyGrid *grid, nature::msg::Odometry odom, double current_steering) {
	state_.x = odom.pose.pose.position.x;
	state_.y = odom.pose.pose.position.y;
	state_.yaw = nature::utils::GetHeadingFromOrientation(odom.pose.pose.orientation);
	state_.v = sqrt(odom.twist.twist.linear.x * odom.twist.twist.linear.x + odom.twist.twist.linear.y * odom.twist.twist.linear.y);
	state_.omega = odom.twist.twist.angular.z;
	state_.steer = current_steering;
	PopulateObstacles(grid);
	// Trajectory log
	std::vector<State> trajectory;
	trajectory.reserve(10000);
	trajectory.push_back(state_);
	State new_state = state_;
	// Main loop (no real-time plotting; write CSV at end)
	int step = 0;
	while (true) {
		auto [u, pred_traj] = DwaControlAck();
		const double v_cmd = u[0];
		const double steer_cmd = u[1];

		new_state = MotionBicycle(new_state, v_cmd, steer_cmd);
		trajectory.push_back(new_state);

		if ((step++ % 5) == 0) {
			std::cout << std::fixed << std::setprecision(3)
				<< "pos=(" << new_state.x << "," << new_state.y << ") "
				<< "yaw=" << new_state.yaw
				<< " v=" << new_state.v
				<< " w=" << new_state.omega
				<< " steer=" << new_state.steer
				<< " | u=(" << v_cmd << ", " << steer_cmd << ")\n";
		}

		// Goal check uses robot_radius for both circle/rectangle (consistent with original)
		const double dist_to_goal = std::hypot(new_state.x - goal_[0], new_state.y - goal_[1]);
		if (dist_to_goal <= config_.robot_radius) {
			std::cout << "Goal!!\n";
			break;
		}

		if (trajectory.size() > 20000) {
			std::cout << "Terminating: too many steps without reaching goal.\n";
			break;
		}
	}
	nature::msg::Path path;
	for (int i = 0; i < (int)trajectory.size(); i++) {
		nature::msg::PoseStamped pose;
		pose.pose.position.x = trajectory[i].x;
		pose.pose.position.y = trajectory[i].y;
		path.poses.push_back(pose);
	}

	return path;
}

} // namespace planning
} // namespace nature