#ifndef AVT_341_UTILS_H
#define AVT_341_UTILS_H

#include "geometry_msgs/Quaternion.h"
#include <tf/transform_datatypes.h>

namespace avt_341 {
namespace utils {

struct vec2{
	vec2(){
		x = 0.0f;
		y = 0.0f;
	}
	vec2(float x_, float y_){
		x = x_; 
		y = y_;
	}
	vec2 operator+(const vec2& b) { return vec2(this->x + b.x, this->y + b.y); }
	vec2 operator-(const vec2& b) { return vec2(this->x - b.x, this->y - b.y); }
	vec2 operator*(const float s) { return vec2(s*this->x, s*this->y); }
	vec2 operator/(const float s) { return vec2(this->x/s, this->y/s); }
	float x;
	float y;
};

inline float length(vec2 p){
	float r = (float)sqrt(p.x*p.x + p.y*p.y);
	return r;
}

inline float dot(vec2 p, vec2 q){
	return (p.x*q.x+p.y*q.y);
}

inline float PointLineDistance(vec2 x1, vec2 x2, vec2 x0) {
	float sx1 = x0.x - x1.x;
	float sy1 = x0.y - x1.y;
	float sx2 = x0.x - x2.x;
	float sy2 = x0.y - x2.y;
	float z = sx1*sy2 - sx2*sy1;
	vec2 x21 = x2 - x1;
	float  d = z / length(x21);
	return d;
}

/**
 * Return distance from a point to a segment
 * \param ep1 First endpoint of the segment
 * \param ep2 Second endpoint of the segment
 * \param p The test point 
 */
inline float PointToSegmentDistance(vec2 ep1, vec2 ep2, vec2 p) {
	vec2 v21 = ep2 - ep1;
	vec2 pv1 = p - ep1;
	if (dot(v21, pv1) <= 0.0) {
		float d = length(pv1);
		return d;
	}
	vec2 v12 = ep1 - ep2;
	vec2 pv2 = p - ep2;
	if (dot(v12, pv2) <= 0.0) {
		float d = length(pv2);
		return d;
	}
	float d0 = PointLineDistance(ep1, ep2, p);
	return d0;
}

inline float GetHeadingFromOrientation(geometry_msgs::Quaternion orientation){
    tf::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
    return (float)yaw;
}

/// Convert any type to a string with zero padding
inline std::string ToString(int x, int zero_padding){
  std::stringstream ss;
  ss << std::setfill('0') << std::setw(zero_padding) << x;
  std::string str = ss.str();
  return str;
};

} //namespace utils
} //namespace avt_341

#endif
