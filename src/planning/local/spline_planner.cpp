#include "nature/planning/local/spline_planner.h"
#include <algorithm>

namespace nature {
namespace planning{
Planner::Planner() {
	// planner coefficients and tuneable parameters
	w_c_ = 0.2f; // comfort
	w_s_ = 0.2f; // safety
	w_d_ = 0.2f; // dynamic safety
	w_r_ = 0.4f; // path deviation
	w_t_ = 0.0f; // terrain segmentation
	alpha_max_ = 5000.0f;
	k_safe_ = 0.8f;
	v_curve_ = 50.0f;
	a_ = 0.01f; // 0.5f;
	b_ = 2.0f;
	averaging_window_size_ = 2;
	// integration step size along the path, meters
	ds_ = 0.1f;
	// state variables to track
	rho_max_ = 1.0f;
	s_max_ = 0.0f;
	first_iter_ = true;
	s_start_ = 0.0f;
	s_no_coll_before_ = 0.0f;
	use_blend_ = true;
}

std::vector<float> Planner::CalcCoeffs(float rho_start, float theta_start, float s_end, float rho_end) {
	std::vector<float> coeffs;
	float d = rho_start;
	float c = (float)tan(theta_start);
	float dp = d - rho_end;
	float se2 = s_end * s_end;
	float b = -(2.0f*c*s_end + 3.0f*dp) / (se2);
	float a = (c*s_end + 2.0f*dp) / (se2*s_end);
	coeffs.push_back(a);
	coeffs.push_back(b);
	coeffs.push_back(c);
	coeffs.push_back(d);
	return coeffs;
}

void Planner::GeneratePaths(int npaths, float s_start, float rho_start, float theta_start, float s_end, float max_steer_angle, float vehicle_width) {
	if (s_end==0) return;
	float lane_width = s_end*tan(max_steer_angle);
	candidates_.clear();
	rho_max_ = lane_width;
	float drho = 2.0f*lane_width / (npaths);
	float rho = 0.5f*drho - lane_width;
	averaging_window_size_ = (int)floor(vehicle_width / drho);
	while (rho <= (lane_width+1.0E-5f)) {
		std::vector<float> coeffs = CalcCoeffs(rho_start, theta_start, s_end, rho);
		Candidate cand(coeffs);
		cand.SetMaxLength(s_end);
		cand.SetS0(s_start);
		candidates_.push_back(cand);
		rho += drho;
	}
	s_max_ = s_end;
	s_start_ = s_start;
}

CurveInfo Planner::InfoOfCurve(Candidate candidate, float s, CurveInfo base_ca) {
	CurveInfo ca;
	float k0 = base_ca.curvature;
	float rho = candidate.At(s);
	float b = 1.0f - rho * k0;
	float B = b / fabs(b);
	float drds = candidate.DerivativeAt(s);
	float drds2 = drds * drds;
	float A = (float)sqrt(drds2 + b * b);
	ca.curvature = (B / A)*(k0 + (b*candidate.SecondDerivativeAt(s)+k0*drds2) / (A*A));
	// info of path_
	double tp = path_.GetTheta(s);
	ca.theta = tp + A*ca.curvature;
	return ca;
}

void Planner::CalculateComfortability() {
	// comfortability and consistency
	for (int i = 0; i < candidates_.size(); i++) {
		float s = 0.0f;
		float comfort = 0.0f;
		float consistent = 0.0f;
		candidates_[i].SetMaxCurvature(0.0f);
		while (s < s_max_) {
			CurveInfo base_ca = path_.GetCurvatureAndAngle(s_start_ + s);
			CurveInfo ca = InfoOfCurve(candidates_[i], s, base_ca);
			if (!first_iter_) {
				CurveInfo last_ca = InfoOfCurve(last_selected_, s, base_ca);
				consistent += (float)sqrt(pow(last_ca.theta - ca.theta, 2.0));
			}
			comfort += ca.curvature*ca.curvature;
			if (fabs(ca.curvature) > candidates_[i].GetMaxCurvature())candidates_[i].SetMaxCurvature(fabs(ca.curvature));
			s += ds_;
		}
		float c_tot = a_ * comfort * ds_ + b_*consistent * ds_ / s_max_;
		candidates_[i].SetComfortability(c_tot);
	}
}

void Planner::CalculateDynamicSafety(nature::msg::Odometry odom) {
	for (int i = 0; i < candidates_.size(); i++) {
		float km = candidates_[i].GetMaxCurvature();
		float vk = (float)sqrt(alpha_max_ / km);
		float fs = candidates_[i].GetStaticSafety();
		float vr = (1.0f - k_safe_ * fs*fs)*v_curve_;
		float v_lim = std::min(vr, vk);
		candidates_[i].SetDynamicSafety(0.0f);
	}
}

void Planner::DilateGrid(nature::msg::OccupancyGrid &grid, int x, float llx, float lly, float urx, float ury){
	//std::cerr << "Grid Size: " << grid.info.width << ", " << grid.info.height << std::endl;
	//std::cerr << "Grid Origin: " << grid.info.origin.position.x << ", " << grid.info.origin.position.y << std::endl;
	//std::cerr << "Grid Resolution: " << grid.info.resolution << std::endl;
	std::vector<int8_t> new_data = grid.data;

	int ix = (int)floor((llx - grid.info.origin.position.x) / grid.info.resolution);
	if(ix < 0) {ix = 0;}
	int iy = (int)floor((lly - grid.info.origin.position.y) / grid.info.resolution);
	if(iy < 0) {iy = 0;}
	int imax_x = (int)ceil((urx - grid.info.origin.position.x) / grid.info.resolution);
	if(imax_x > grid.info.width) {imax_x = grid.info.width;}
	int imax_y = (int)ceil((ury - grid.info.origin.position.y) / grid.info.resolution);
	if(imax_y > grid.info.height) {imax_x = grid.info.height;}
	//std::cerr << "Dilate Grid: (" << ix << ", " << iy << ") to (" << imax_x << ", " << imax_y << ")" << std::endl;

	for (int i=ix+x;i<imax_x-x;i++){
		for (int j=iy+x;j<imax_y-x;j++){
			int n = i*grid.info.height+j;
			for (int ii=-x;ii<=x;ii++){
				for (int jj=-x;jj<=x;jj++){
					int iii = i+ii;
					int jjj = j+jj;
					int nd = iii*grid.info.height+jjj;
					new_data[n] = std::max(new_data[n],grid.data[nd]);
				}
			}
		}
	}
	grid.data = new_data;
}

void Planner::CalculateStaticSafetyAndSegCost(const nature::msg::OccupancyGrid & grid, const nature::msg::OccupancyGrid & grid_seg) {
	bool has_segmentation = grid_seg.info.height>0 && grid_seg.info.width>0;
	for (int i = 0; i < candidates_.size(); i++) {
		float s = s_no_coll_before_;
		float stat_safe = 0.0f;
		float traj_seg_cost = 0.0;
		while (s < s_max_) {
			float rho = candidates_[i].At(s);
			if (fabs(rho) > rho_max_)candidates_[i].SetOutOfBounds(true);
			utils::vec2 p = path_.ToCartesian(s_start_ + s, rho);
			int ix = (int)floor((p.x - grid.info.origin.position.x) / grid.info.resolution);
			int iy = (int)floor((p.y - grid.info.origin.position.y) / grid.info.resolution);
			if (ix >= 0 && ix < (int)grid.info.width && iy >= 0 && iy < (int)grid.info.height) {
				int ndx = ix * grid.info.height + iy;
				stat_safe += grid.data[ndx];
				traj_seg_cost += (has_segmentation ? grid_seg.data[ndx] : 0.0f);
			}
			s += ds_;
		}
		if (stat_safe > 0) {
			candidates_[i].SetHitsObstacle(true);
			candidates_[i].SetStaticSafety(1.0f);
		}
		else {
			candidates_[i].SetHitsObstacle(false);
			candidates_[i].SetStaticSafety(0.0f);
		}
		candidates_[i].SetSegmentationCost(traj_seg_cost);
	}

	// now blend
	if(use_blend_){
		std::vector<float> fs;
		std::vector<float> fseg;
		fs.resize(candidates_.size(),0.0f);
		fseg.resize(candidates_.size(),0.0f);
		for (int i = 0; i < candidates_.size(); i++) {
		float fcount = 0.0f;
		for (int k = -averaging_window_size_; k <= averaging_window_size_; k++) {
			int ndx = i + k;
			if (ndx >= 0 && ndx < candidates_.size()) {
			fs[i] += candidates_[ndx].GetStaticSafety();
			fseg[i] += candidates_[ndx].GetSegmentationCost();
			fcount += 1.0f;
			}
		}
		fs[i] = fs[i] / fcount;
		fseg[i] = fseg[i] / fcount;
		}
		for (int i = 0; i < candidates_.size(); i++) {
		candidates_[i].SetStaticSafety(fs[i]);
		candidates_[i].SetSegmentationCost(fseg[i]);
		}
	}
}

void Planner::CalculateRhoCost() {
	for (int i = 0; i < candidates_.size(); i++) {
		float rho_final = candidates_[i].At(s_max_);
		float rho_cost = (float)fabs(rho_final / rho_max_);
		candidates_[i].SetRhoCost(rho_cost);
	}
}

float Planner::GetTotalCostOfCandidate(int i) {
	float cost = w_c_ * candidates_[i].GetComfortability() + w_s_ * candidates_[i].GetStaticSafety() + w_r_ * candidates_[i].GetRhoCost() + w_d_*candidates_[i].GetDynamicSafety() * w_t_*candidates_[i].GetSegmentationCost();
  	candidates_[i].SetCost(cost);
	return cost;
}

bool Planner::CalculateCandidateCosts(nature::msg::OccupancyGrid grid, nature::msg::OccupancyGrid segmentation_grid, nature::msg::Odometry odom) {

	CalculateStaticSafetyAndSegCost(grid, segmentation_grid);
	CalculateComfortability();
	CalculateRhoCost();
	CalculateDynamicSafety(odom);

	int lowest_index = -1;
	float lowest_cost = std::numeric_limits<float>::max();
	for (int i = 0; i < candidates_.size(); i++) {
		float cost = GetTotalCostOfCandidate(i);
		if (cost < lowest_cost && !candidates_[i].HitsObstacle() && !candidates_[i].IsOutOfBounds()) {
			lowest_cost = cost;
			lowest_index = i;
		}
	}
	if (lowest_index == -1) { // pick a path that leaves the lane
		for (int i = 0; i < candidates_.size(); i++) {
			float cost = GetTotalCostOfCandidate(i);
			if (cost < lowest_cost && !candidates_[i].HitsObstacle()) {
				lowest_cost = cost;
				lowest_index = i;
			}
		}
	}

	if (lowest_index == -1) {
		return false;
	}
	candidates_[lowest_index].SetRank(1);
	last_selected_ = candidates_[lowest_index];
	first_iter_ = false;
	return true;
}

utils::vec2 Planner::GetNextPoint(float s_step) {
	utils::vec2 point(0.0f, 0.0f);
	if (!first_iter_) {
		float rho = last_selected_.At(s_step);
		point = path_.ToCartesian(s_start_ + s_step, rho);
	}
	return point;
}

float Planner::GetAngleAt(float s) {
	float theta = 0.0f;
	if (!first_iter_) {
		CurveInfo base_ca = path_.GetCurvatureAndAngle(s);
		CurveInfo ca = InfoOfCurve(last_selected_, s, base_ca);
		theta = ca.theta;
	}
	return theta;
}

} // namespace planning
} // namespace nature