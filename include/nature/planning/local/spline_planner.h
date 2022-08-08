/**
 * \class Path
 *
 * Class for the path planner. 
 * Adapated for use in off-road with ROS from the paper:
 * 
 * Hu, X., Chen, L., Tang, B., Cao, D., & He, H. (2018). 
 * Dynamic path planning for autonomous driving on various roads with avoidance of static and moving obstacles. 
 * Mechanical Systems and Signal Processing, 100, 482-500.
 *
 * \author Chris Goodin
 *
 * \date 9/3/2020
 */
#ifndef SPLINE_PLANNER_H
#define SPLINE_PLANNER_H

#include <vector>
#include "nature/planning/local/spline_path.h"
#include "nature/planning/local/candidate.h"
// ROS INCLUDES
#include "nature/node/ros_types.h"

namespace nature {
namespace planning{

class Planner {
public:
	/**
	 * Create an empty planner.
	 */ 
	Planner();

	/**
	 * Set the desired centerline for the planner.
	 * \param path A tang_planner::Path object. 
	 */
	void SetCenterline(Path path) { path_ = path; }

	/**
	 * Generate a set of candidate paths.
	 * \param npaths The number of paths to generate.
	 * \param s_start The arc length along the centerline at which to start.
	 * \param rho_start The offset from the path in the initial configuration.
	 * \param theta_start The angle of the vehicle relative to east in the initial configuration.
	 * \param s_look_ahead Distance (path length) to plan in the forward direction.
	 * \param max_steer_angle The maximum steering angle of the vehicle, radians
	 * \param vehicle_width The width of the vehicle, in meters.
	 */
	void GeneratePaths(int npaths, float s_start, float rho_start, float theta_start, float s_look_ahead, 
	float max_steer_angle, float vehicle_width);

	/**
	 * Get a list of the candidate paths.
	 */ 
	std::vector<Candidate> GetCandidates() { return candidates_; }

	/**
	 * Calculate a list of candidate costs given an occupancy grid and vehicle odometry.
	 * \param grid ROS occupancy grid.
	 * \param odom ROS odometry of the current vehicle.
	 */ 
	bool CalculateCandidateCosts(nature::msg::OccupancyGrid grid, nature::msg::OccupancyGrid segmentation_grid, nature::msg::Odometry odom);

	/**
	 * Dilate the map with a mask of given size.
	 * \param grid The occupancy grid to dilate.
	 * \param x The dilation mask size is (x+1)*(x+1).
	 */
	void DilateGrid(nature::msg::OccupancyGrid &grid, int x, float llx, float lly, float urx, float ury);

	/**
	 * Get a point along the optimal path at an arc length s_step from the current position. 
	 */
	utils::vec2 GetNextPoint(float s_step);

	/**
	 * Get the angle at arc length s along the optimal path. 
	 */
	float GetAngleAt(float s);

	/**
	 * Return the optimal path. 
	 */
	Candidate GetBestPath(){return last_selected_;}

	/**
	 * Set the weight on the comfortability factor. 
	 * Default is w_c = 0.2
	 * \param w Desired weight.
	 */ 
	void SetComfortabilityWeight(float w){ w_c_ = w; }

	/**
	 * Set the weight on the static safety factor. 
	 * Default is w_s = 0.2
	 * \param w Desired weight.
	 */ 
	void SetStaticSafetyWeight(float w){ w_s_ = w; }

	/**
	 * Set the weight on the dynamic safety factor. 
	 * Default is w_d = 0.2
	 * \param w Desired weight.
	 */ 
	void SetDynamicSafetyWeight(float w){ w_d_ = w; }

	/**
	 * Set the weight on the path adherence factor. 
	 * Default is w_r = 0.4
	 * \param w Desired weight.
	 */ 
	void SetPathAdherenceWeight(float w){ w_r_ = w; }

	/**
	 * Sets wether or not to use blending during local planning. 
	 * Blending will blend cost of i'th candidate trajectory based on adjacent candidate paths within vehicle width.
	 * Default use_blend = true
	 * \param use_blend Whether to use blending or not.
	 */ 
	void SetIgnoreCollBeforeDist(float s_no_coll_before) { s_no_coll_before_ = s_no_coll_before; }

	/**
	 * Sets wether or not to use blending during local planning. 
	 * Blending will blend cost of i'th candidate trajectory based on adjacent candidate paths within vehicle width.
	 * Default use_blend = true
	 * \param use_blend Whether to use blending or not.
	 */ 
  	void SetUseBlend(bool use_blend){ use_blend_ = use_blend; }

	float GetComfortabilityWeight() const { return w_c_; }
	float GetStaticSafetyWeight() const { return w_s_; }
	float GetDynamicSafetyWeight() const { return w_d_; }
	float GetPathAdherenceWeight() const { return w_r_; }
	float GetSegmentationWeight() const { return w_t_; }

	/**
	 * Set the weight on the consistency factor on the comfortability calculation. 
	 * Default is b = 2.0
	 * \param w Desired weight.
	 */ 
	void SetConsistencyFactorWeight(float w){ b_= w; }

    /**
    * Set the weight on the terrain segmentation cost.
    * Default is w_t = 0.00
    * \param w Desired weight.
    */
    void SetSegmentationFactorWeight(float w){ w_t_ = w; }

	/**
	 * Set the weight on the curvature factor on the comfortability calculation. 
	 * Default is a = 0.01
	 * \param w Desired weight.
	 */ 
	void SetCurvatureFactorWeight(float w){ a_ = w; }

	/**
	 * Set the size of the averaging window for static safety, in number of paths.
	 * Default is calculated by the vehicle width
	 * \param np Number of paths.
	 */ 
	void SetAveragingWindowSize(int np){ averaging_window_size_ = np; }

	/**
	 * Set the integration step size for curvature calcuations and other integrations.
	 * Default is ds = 0.1 meters
	 * \param ds The integration step size. 
	 */
	void SetArcLengthIntegrationStep(float ds){ ds_ = ds; }

	/**
	 * Set the dynamic safety factors. See equations 19-20 of 
	 * Hu et al. for further details.
	 * 
	 * \param alpha Limit of lateral acceleration, default = 5000.0
	 * \param k Safety gain for speed adjustment, default = 0.8
	 * \param v Reference speed for the path, default = 50.0
	 */ 
	void SetDynamicSafetyParams(float alpha, float k, float v){
		alpha_max_ = alpha;
		k_safe_ = k;
		v_curve_ = v;
	}

private:
	// private methods
	std::vector<float> CalcCoeffs(float rho_start, float theta_start, float s_end, float rho_end);
	void CalculateComfortability();
	void CalculateStaticSafetyAndSegCost(const nature::msg::OccupancyGrid & grid,const nature::msg::OccupancyGrid & segmentation_grid);
	void CalculateRhoCost();
	void CalculateDynamicSafety(nature::msg::Odometry odom);
	float GetTotalCostOfCandidate(int pathnum);
	CurveInfo InfoOfCurve(Candidate candidate, float s, CurveInfo base_ca);

	// centerline
	Path path_;

	// candidates
	std::vector<Candidate> candidates_;

	// optimal path
	Candidate last_selected_;

	// state variables to track
	bool first_iter_;
	float s_max_;
	float rho_max_;
	float s_start_;

	// Planner parameters
	float w_c_;
	float w_s_;
	float w_d_;
	float w_r_;
	float w_t_;
	float alpha_max_; 
	float k_safe_;
	float v_curve_;
	float a_;
	float b_;
	float ds_;
	float s_no_coll_before_;
	int averaging_window_size_;
	bool use_blend_;
};

} // namespace planning
} // namespace nature

#endif