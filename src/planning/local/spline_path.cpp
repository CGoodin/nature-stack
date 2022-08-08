#include "nature/planning/local/spline_path.h"

namespace nature {
namespace planning{

Path::Path() {
	max_lookahead_ = std::numeric_limits<float>::max();
}

Path::Path(std::vector<utils::vec2> points) {
	Init(points);
}

Path::Path(std::vector<utils::vec2> points, utils::vec2 position, float la) {
	Init(points, position, la);
}

void Path::Init(std::vector<utils::vec2> points, utils::vec2 position, float la){
	// cull points
	std::vector<utils::vec2> points_to_keep; 
	for (int i=0;i<points.size();i++){
		float ds = utils::length(position-points[i]);
		if (ds>0.0 && ds<la) points_to_keep.push_back(points[i]);
	}
	max_lookahead_ = la;
	Init(points_to_keep);
}

void Path::Init(std::vector<utils::vec2> points) {
	points_ = points;
	CalcAnglesAndCurvature();
}


float Path::TriangleArea(utils::vec2 a, utils::vec2 b, utils::vec2 c) {
	float area = (float)fabs(a.x*(b.y - c.y) + b.x*(c.y - a.y) + c.x*(a.y - b.y));
	return area;
}

float Path::MengerCurvature(utils::vec2 a, utils::vec2 b, utils::vec2 c) {
	float curv = 0.0f;
	float denom = utils::length(a - b)*utils::length(b - c)*utils::length(c - b);
	if (denom == 0.0f) {
		curv = std::numeric_limits<float>::max();
	}
	else {
		float area = TriangleArea(a, b, c);
		curv = 4.0f*area / denom;
	}
	return curv;
}

void Path::CalcAnglesAndCurvature() {
	curvature_.resize(points_.size(), 0.0f);
	theta_.resize(points_.size(), 0.0f);
	arc_length_.resize(points_.size(),0.0f);
	discrete_lengths_.resize(points_.size(),0.0f);
	if (points_.size() > 2) {
		for (int i = 1; i < points_.size() - 1; i++){
		curvature_[i] = MengerCurvature(points_[i - 1], points_[i], points_[i + 1]);
		}
		curvature_[0] = curvature_[1];
		curvature_[points_.size() - 1] = curvature_[points_.size() - 2];
	}
	for (int i = 0; i < points_.size()-1; i++) {
		utils::vec2 v1 = points_[i + 1] - points_[i];
		discrete_lengths_[i] = utils::length(v1);
	}
	//angle and arc length
	for (int i = 1; i < points_.size()-1; i++) {
		utils::vec2 v0 = points_[i] - points_[i - 1];
		utils::vec2 v1 = points_[i + 1] - points_[i];
		float sl = utils::length(v0);
		v0 = v0 / sl; //utils::length(v0);
		v1 = v1 / utils::length(v1);
		arc_length_[i] = arc_length_[i-1]+sl;
		float theta0 = (float)atan2(v0.y, v0.x);
		float theta1 = (float)atan2(v1.y, v1.x);
		theta_[i] = 0.5f*(theta0 + theta1);
	}
	// do the angle for the first and last point
	if (points_.size() > 2) {
		utils::vec2 v_first = points_[1] - points_[0];
		v_first = v_first / utils::length(v_first);
		utils::vec2 v_last = points_[points_.size() - 1] - points_[points_.size() - 2];
		float sl = utils::length(v_last);
		v_last = v_last / sl; //utils::length(v_last);
		arc_length_[points_.size()-1] = arc_length_[points_.size()-2] + sl;
		theta_[0] = (float)atan2(v_first.y, v_first.x);
		theta_[points_.size() - 1] = (float)atan2(v_last.y, v_last.x);
	}
}

float Path::GetTheta(float s){
	SegmentInfo seg = FindSegment(s);
	return theta_[seg.id];
}

PointSegDist Path::PointToSegmentDistance(utils::vec2 P, utils::vec2 Q, utils::vec2 X) {
	// https ://diego.assencio.com/?index=ec3d5dfdfc0b6a0d147a656f0af332bd
	utils::vec2 XP = X - P;  
	utils::vec2 QP = Q - P; 
	PointSegDist pseg;
	float disc = utils::dot(QP, QP);
	if (disc == 0.0f) {
		pseg.dist = utils::length(P - X);
		pseg.point = P;
		return pseg;
	}
	float ls = utils::dot(XP, QP) / disc;
	utils::vec2 S;
	if (ls <= 0.0f) {
		S = P;
	}
	else if (ls >= 1.0f) {
		S = Q;
	}
	else {
		S = P + QP*ls;
	}
	pseg.dist = utils::length(S - X);
	pseg.point = S;
	return pseg;
}

SegmentInfo Path::FindSegment(float s) {
	SegmentInfo segment;
	/*
	float cum_dist = 0.0;
	int wp = 0;
	segment.id = 0; 
	while (cum_dist < s && wp < points_.size() - 1) {
		float d = utils::length(points_[wp] - points_[wp + 1]);
		if (d + cum_dist >= s) {
			segment.id = wp;
			float t = s - cum_dist;
			utils::vec2 v = points_[wp + 1] - points_[wp];
			v = v / utils::length(v);
			segment.point = points_[wp] + v * t; 
			break;
		}
		cum_dist = cum_dist + d;
		wp = wp + 1;
	}
	*/	
	bool found = false;
	for (int i=1;i<arc_length_.size();i++){
		if (arc_length_[i]>s){
			segment.id = i-1;
			float t = s - arc_length_[i-1];
			utils::vec2 v = points_[i] - points_[i-1];
			v = v / utils::length(v);
			segment.point = points_[i] + v * t; 
			found = true;
			break;
		}
	}	
	if (!found){
		int i = arc_length_.size()-1;
		segment.id = i-1;
		float t = s - arc_length_[i-1];
		utils::vec2 v = points_[i] - points_[i-1];
		v = v / utils::length(v);
		segment.point = points_[i] + v * t; 
	}

	return segment;
}

float Path::GetTotalLength() {
	float cum_dist = 0.0;
	for (int i = 0; i < points_.size() - 1; i++) {
		cum_dist += utils::length(points_[i] - points_[i + 1]);
	}
	return cum_dist;
}

void Path::FixBeginning(float x, float y){
	utils::vec2 sr = ToSRho(x,y);
	float s = sr.x;
	while (s<=0.0f){
		utils::vec2 extend_direction = points_[0] - points_[1];
		extend_direction = extend_direction/utils::length(extend_direction);
		utils::vec2 new_point = points_[0] + extend_direction * 100.0f;
		points_.insert(points_.begin(),new_point);
		Init(points_);
		utils::vec2 sr0 = ToSRho(x,y);
		s = sr0.x;
	}
}

void Path::FixEnd(){
	int np = (int)(points_.size());
	utils::vec2 extend_direction = points_[np-1] - points_[np-2];
	float wp_dist = utils::length(extend_direction);
	extend_direction = extend_direction/wp_dist;
	int num_to_add = (int)(200.0f/wp_dist);
	if (num_to_add<1)num_to_add = 1;
	for (int i=1;i<num_to_add+1; i++){
		utils::vec2 new_point = points_[np-1] + extend_direction * i*wp_dist;
		points_.push_back(new_point);
	}
	Init(points_);
}


utils::vec2 Path::ToSRho(float x, float y) {
	// First find the closest segment
	int closest_index = 0;
	float closest = std::numeric_limits<float>::max();
	utils::vec2 tp(x, y);
	utils::vec2 closest_point;
	float dist_sign = 1.0f;
	for (int i = 0; i < points_.size() - 1; i++) {
		utils::vec2 seg = points_[i + 1] - points_[i];
		PointSegDist d = PointToSegmentDistance(points_[i], points_[i + 1], tp);
		if (d.dist < closest) {
			utils::vec2 v = tp - points_[i];
			float sign =  v.y*seg.x - v.x*seg.y;
			float smag = fabs(sign);
			if (smag>0.0f) dist_sign = sign / fabs(sign);
			closest = d.dist;
			closest_point = d.point;
			closest_index = i;
		}
	}
	
	float rho = dist_sign*closest;
	float s = 0.0f;
	for (int i = 0; i < closest_index; i++) {
		s += utils::length(points_[i] - points_[i + 1]);
	}

	s += utils::length(closest_point - points_[closest_index]);

	utils::vec2 sr;
	sr.x = s;
	sr.y = rho;
	return sr;
}

utils::vec2 Path::ToCartesian(float s, float rho) {
	float cumulative_distance = 0.0f;
	int segment = 0;
	while (cumulative_distance<s && segment<(points_.size()-1)){
		float arc_length = discrete_lengths_[segment];
		if (cumulative_distance+arc_length>s){
			break;
		}
		else{
			cumulative_distance+=arc_length;
			segment++;
		}
	}
	utils::vec2 v = points_[segment + 1] - points_[segment];
	v = v / utils::length(v);
	utils::vec2 n(-v.y, v.x);
	utils::vec2 p = points_[segment] + v*(s-cumulative_distance)+n*rho;
	return p;
}

CurveInfo Path::GetCurvatureAndAngle(float s) {
	SegmentInfo seg = FindSegment(s);
	float d0 = utils::length(seg.point - points_[seg.id]);
	float d1 = utils::length(seg.point - points_[seg.id + 1]);
	CurveInfo ca;
	if (d0 == 0.0f) {
		ca.curvature = curvature_[seg.id];
		ca.theta = theta_[seg.id];
	}
	else if (d1 == 0.0f) {
		ca.curvature = curvature_[seg.id + 1];
		ca.theta = theta_[seg.id + 1];
	}
	else {
		float w0 = 1.0f / d0;
		float w1 = 1.0f / d1;
		float w = w0 + w1;
		ca.curvature = (w0*curvature_[seg.id] + w1 * curvature_[seg.id + 1]) / w;
		ca.theta = (w0*theta_[seg.id] + w1 * theta_[seg.id + 1]) / w;
	}
	return ca;
}

} // namespace planning
} // namespace nature