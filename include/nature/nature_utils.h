/**
 * \file nature_utils.h
 *
 * Structs and inline functions used by all the algorithms.
 *
 * \date 9/3/2020
 */
#ifndef NATURE_UTILS_H
#define NATURE_UTILS_H

#include "nature/node/ros_types.h"
#include <iomanip>
#include <sstream>

namespace nature {
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

struct vec3{
	vec3(){
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
	}
	vec3(float x_, float y_, float z_){
		x = x_; 
		y = y_;
		z = z_;
	}
	vec3 operator+(const vec3& b) { return vec3(this->x + b.x, this->y + b.y, this->z + b.z); }
	vec3 operator-(const vec3& b) { return vec3(this->x - b.x, this->y - b.y, this->z - b.z); }
	vec3 operator*(const float s) { return vec3(s*this->x, s*this->y, s*this->z); }
	vec3 operator/(const float s) { return vec3(this->x/s, this->y/s, this->z/s); }
  float operator[](const int idx) const {
    return idx == 0 ? x : (idx == 1 ? y : z);
  };
	float x;
	float y;
	float z;
};

struct ivec2{
	ivec2(){
		x = 0;
		y = 0;
	}
	ivec2(int x_, int y_){
		x = x_; 
		y = y_;
	}
	ivec2 operator+(const ivec2& b) { return ivec2(this->x + b.x, this->y + b.y); }
	ivec2 operator-(const ivec2& b) { return ivec2(this->x - b.x, this->y - b.y); }
	ivec2 operator*(const int s) { return ivec2(s*this->x, s*this->y); }
	ivec2 operator/(const int s) { return ivec2(this->x/s, this->y/s); }
	int x;
	int y;
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

inline float GetHeadingFromOrientation(nature::msg::Quaternion orientation){
    nature::msg_tf::Quaternion q(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w);
    nature::msg_tf::Matrix3x3 m(q);
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
} //namespace nature

#endif
