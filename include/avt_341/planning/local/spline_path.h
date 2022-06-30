/**
 * \class Path
 *
 * Path class for the planner. This is the 
 * equivalent of the centerline in the original planner.
 *
 * \author Chris Goodin
 *
 * \date 8/31/2020
 */
#ifndef SPLINE_PATH_H
#define SPLINE_PATH_H

#include <vector>
#include "avt_341/avt_341_utils.h"

namespace avt_341 {
namespace planning{
	
/// Info regarding a path segment.
struct SegmentInfo {
	utils::vec2 point;
	int id;
};

/// Distance from a point to a segment, and the closest point on the segment.
struct PointSegDist {
	utils::vec2 point;
	float dist;
};

/// Curvature and tangent angle of the path.
struct CurveInfo {
	float curvature;
	float theta;
};

class Path {
public:
	/**
	 * Create an empty path.
	 */ 
	Path();

	/**
	 * Create a path and initialize it with a list of waypoints.
	 * \param points List of waypoints in 2D ENU coordinates.
	 */ 
	Path(std::vector<utils::vec2> points);

	/**
	 * Create a path and initialize it with a list of waypoints.
	 * Will cull out waypoints that are far away to make the calculation faster.
	 * \param points List of waypoints in 2D ENU coordinates.
	 * \param position Current position in 2D ENU coordinates.
	 * \param la The maximum distances ahead on the current position to keep.
	 */ 
	Path(std::vector<utils::vec2> points, utils::vec2 position, float la);

	/**
	 * Initialize a path with a list of waypoints.
	 * \param points List of waypoints in 2D ENU coordinates.
	 */ 
	void Init(std::vector<utils::vec2> points);

	/**
	 * Initialize a path with a list of waypoints. 
	 * Will cull out waypoints that are far away to make the calculation faster.
	 * \param points List of waypoints in 2D ENU coordinates.
	 * \param position Current position in 2D ENU coordinates.
	 * \param la The maximum distances ahead on the current position to keep.
	 */ 
	void Init(std::vector<utils::vec2> points, utils::vec2 position, float la);

	/**
	 * Get the total arc length of the path, from the first to the last waypoint.
	 */ 
	float GetTotalLength();

	/**
	 * Convert a point in the s-rho coordinate system to Cartesian coordinates.
	 * \param s The arc length parameter.
	 * \param rho The offset parameter. 
	 */
	utils::vec2 ToCartesian(float s, float rho);

	/**
	 * Convert a point from Cartesian coordinates to the s-rho system.
	 * \param x The x-coordinate in local ENU.
	 * \param y The y-coordinate in local ENU.
	 */ 
	utils::vec2 ToSRho(float x, float y);

	/**
	 * Get the curvature and tangent angle at a given arc length along the path.
	 * \param s The arc length and which to measure the curvature.
	 */ 
	CurveInfo GetCurvatureAndAngle(float s);

	/**
	 * Get the last point on the path. 
	 */
	utils::vec2 GetLastPoint() { return points_[points_.size() - 1]; }

	/**
	 * Get point of a given index.
	 * \param index The index of the point to get. 
	 */
	utils::vec2 GetPoint(int index) {
		utils::vec2 p(0.0f, 0.0f);
		if (index >= 0 && index < points_.size()) {
			p = points_[index];
		}
		return p;
	}

	/**
	 * Return the list of points on the path.
	 */ 
	std::vector<utils::vec2> GetPoints(){
		return points_;
	}

	/**
	 * Get the angle from X at a given path length.
	 * \param s The arc length along the path at which to find the angle.
	 */
	float GetTheta(float s);

	/**
	* Extend the road centerline behind the vehicle start position
	* \param x Vehicle start X in local ENU
	* \param y Vehicle start Y in local ENU
	*/
	void FixBeginning(float x, float y);

	/**
	* Extend the road centerline beyond the final waypoint
	*/
	void FixEnd();

private:
	std::vector<utils::vec2> points_;
	std::vector<float> curvature_;
	std::vector<float> theta_;
	std::vector<float> arc_length_;
	std::vector<float> discrete_lengths_;
	float max_lookahead_;
	void CalcAnglesAndCurvature();

	float MengerCurvature(utils::vec2 p0, utils::vec2 p1, utils::vec2 p2);
	float TriangleArea(utils::vec2 a, utils::vec2 b, utils::vec2 c);

	PointSegDist PointToSegmentDistance(utils::vec2 P, utils::vec2 Q, utils::vec2 X);
	SegmentInfo FindSegment(float s);

};

} // namespace planning
} // namespace avt_341


#endif