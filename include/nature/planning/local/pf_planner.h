/**
 * \class PfPlanner
 *
 * Class for the potential field planner. 
 * 
 * author: Atsushi Sakai (@Atsushi_twi)
 * Ref:
 * https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
 *
 * Original source code from:
 * https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/PotentialFieldPlanning\
 *
 * Modified by CTG to be faster, integrate with ROS, and work with an Occupancy grid.
 *
 * \author Chris Goodin
 *
 * \date 1/19/2022
 */
#ifndef PF_PLANNER_H
#define PF_PLANNER_H
// c++ includes
#include <vector>
// ROS INCLUDES
#include "nature/node/ros_types.h"

namespace nature {
namespace planning{

class PfPlanner {
public:
	/**
	 * Create an empty planner.
	 */ 
	PfPlanner();

	/**
	 * Calculate a list of candidate costs given an occupancy grid and vehicle odometry.
	 * \param grid ROS occupancy grid.
	 * \param odom ROS odometry of the current vehicle.
	 */ 
	nature::msg::Path Plan(nature::msg::OccupancyGrid grid, nature::msg::Odometry odom);

	/**
	 * Set the goal point in the local ENU coordinate frame
	 */
	void SetGoal(float gx, float gy);

	/// Set the Eta parameter
	void SetEta(float eta){eta_ = eta;}

	/// Set the Kp parameter
	void SetKp(float kp){kp_ = kp;}

	/// Set the cutoff distance
	void SetCutoffDistance(float cutoff_dist){ obs_cutoff_dist_ = cutoff_dist; }

private:
	float Hypot(float x, float y);
    
	float CalcAttractivePotential(float x, float y, float gx, float gy);

	float CalcRepulsivePotential(float x, float y, std::vector<float> ox, std::vector<float> oy);

	std::vector<std::vector<float> > GetMotionModel(float step);

	void PotentialFieldPlanning(float minx, float miny, float reso, float sx, float sy, float gx, float gy, std::vector<float> ox, std::vector<float> oy);

	float goal_x_;
	float goal_y_;
	float kp_;
	float eta_;
	float obs_cutoff_dist_;
	std::vector<float> rx_;
	std::vector<float> ry_;
};

} // namespace planning
} // namespace nature

#endif