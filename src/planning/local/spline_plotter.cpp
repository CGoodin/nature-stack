
#include "avt_341/planning/local/spline_planner.h"
#include "avt_341/planning/local/spline_plotter.h"
#include "avt_341/avt_341_utils.h"
#include <limits>

namespace avt_341 {
namespace planning{

Plotter::Plotter() {
	nx_ = 256;
	ny_ = 256;
	pixdim_ = 1.0f;
	x_lo_ = -128.0f;
	x_hi_ = 128.0f;
	y_lo_ = -128.0f;
	y_hi_ = 128.0f;
	map_set_ = false;
}

void Plotter::AddMap(nav_msgs::OccupancyGrid grid){
	grid_ = grid;
	x_lo_ = grid.info.origin.position.x;
	y_lo_ = grid.info.origin.position.y;
	if (grid.info.width!=nx_ || grid.info.height!=ny_){
		nx_ = grid.info.width;
		ny_ = grid.info.height;
		disp_.resize(nx_,ny_);
	}
	pixdim_ = grid.info.resolution;
	x_hi_ = x_lo_ + nx_*pixdim_;
	y_hi_ = y_lo_ + ny_*pixdim_;
	map_set_ = true;
}

void Plotter::SetPath(std::vector<avt_341::utils::vec2> path) {
	path_ = path;
}

void Plotter::AddWaypoints(nav_msgs::Path waypoints){
	waypoints_.clear();
	for (int i=0;i<waypoints.poses.size();i++){
		avt_341::utils::vec2 p;
		p.x = waypoints.poses[i].pose.position.x;
		p.y = waypoints.poses[i].pose.position.y;
		waypoints_.push_back(p);
	}
}

void Plotter::AddCurves(std::vector<Candidate> curves) {
	curves_ = curves;
}

avt_341::utils::ivec2 Plotter::CartesianToPixel(float x, float y) {
	avt_341::utils::ivec2 pix;
	pix.x = (int)floor((x - x_lo_) / pixdim_);
	pix.y = (int)floor((y - y_lo_) / pixdim_);
	pix.x = std::min(std::max(0, pix.x), nx_);
	pix.y = std::min(std::max(0, pix.y), ny_);
	return pix;
}

void Plotter::Display(){
	Display(false,"",nx_,ny_);
}

void Plotter::Display(bool save, std::string ofname, int nx, int ny) {
	if (!map_set_)return;
	cimg_library::CImg<float> image;
	image.assign(nx_, ny_, 1, 3, 0.0f);

	avt_341::utils::vec3 white(255.0f, 255.0f, 255.0f);
	avt_341::utils::vec3 red(255.0f, 0.0f, 0.0f);
	avt_341::utils::vec3 green(0.0f, 255.0f, 0.0f);
	avt_341::utils::vec3 yellow(255.0f, 255.0f, 0.0f);
	avt_341::utils::vec3 orange(255.0f, 165.0f, 0.0f);
	avt_341::utils::vec3 blue(0.0f, 0.0f, 255.0f);
	avt_341::utils::vec3 purple(255.0f, 0.0f, 255.0f);

	// Add the occupancy grid
	for (int i = 0; i < nx_; i++) {
		float x = x_lo_ + (i + 0.5f)*pixdim_;
		int idx = (int)floor((x - grid_.info.origin.position.x) / grid_.info.resolution);
		for (int j = 0; j < ny_; j++) {
			float y = y_lo_ + (j + 0.5f)*pixdim_;
			int idy = (int)floor((y - grid_.info.origin.position.y) / grid_.info.resolution);
			if (idx >= 0 && idx < (int)grid_.info.width && idy >= 0 && idy <= (int)grid_.info.height) {
				int n = idx * grid_.info.height + idy;
				if (grid_.data[n] > 0) {
					image.draw_point(i,j, (float *)&red);
				}
			}
		}
	}

	// plot waypoints
	for (int i = 0; i < waypoints_.size(); i++) {
		avt_341::utils::ivec2 pix = CartesianToPixel(waypoints_[i].x, waypoints_[i].y);
		image.draw_circle(pix.x, pix.y, 2, (float *)&white);
		if (i < waypoints_.size() - 1) {
			avt_341::utils::ivec2 pix1 = CartesianToPixel(waypoints_[i+1].x, waypoints_[i+1].y);
			image.draw_line(pix.x, pix.y, pix1.x, pix1.y, (float *)&white);
		}
	}

	// plot global path
	for (int i = 0; i < path_.size(); i++) {
		avt_341::utils::ivec2 pix = CartesianToPixel(path_[i].x, path_[i].y);
		if (i < path_.size() - 1) {
			avt_341::utils::ivec2 pix1 = CartesianToPixel(path_[i+1].x, path_[i+1].y);
			image.draw_line(pix.x, pix.y, pix1.x, pix1.y, (float *)&orange);
		}
	}

	float ds = pixdim_;
	Path wp_path(path_);
	for (int i = 0; i < curves_.size(); i++) {
		float s0 = curves_[i].GetS0() + ds;
		float s_max = s0 + curves_[i].GetMaxLength() - ds;
		while (s0 < s_max){
			float rho0 = curves_[i].At(s0- curves_[i].GetS0());
			float s1 = s0 + pixdim_;
			float rho1 = curves_[i].At(s1- curves_[i].GetS0());
			avt_341::utils::vec2 pc0 = wp_path.ToCartesian(s0, rho0);
			avt_341::utils::vec2 pc1 = wp_path.ToCartesian(s1, rho1);
			avt_341::utils::ivec2 p0 = CartesianToPixel(pc0.x, pc0.y);
			avt_341::utils::ivec2 p1 = CartesianToPixel(pc1.x, pc1.y);
			if (!(std::isnan(pc0.x) || std::isnan(pc0.y) ||
			 std::isnan(pc1.x) || std::isnan(pc1.y)  )){
				if (curves_[i].GetRank() == 1) {
					image.draw_line(p0.x, p0.y, p1.x, p1.y, (float *)&green);
				}
				else if (curves_[i].HitsObstacle()) {
					image.draw_line(p0.x, p0.y, p1.x, p1.y, (float *)&red);
				}
				else if (curves_[i].IsOutOfBounds()) {
					image.draw_line(p0.x, p0.y, p1.x, p1.y, (float *)&yellow);
				}
				else {
					image.draw_line(p0.x, p0.y, p1.x, p1.y, (float *)&blue);			
				}
			}
			s0 += pixdim_;
		}
	}

	image.mirror('y');

	if (save){
		image.resize(nx,ny);
		image.save(ofname.c_str());
	}

	disp_ = image;
};

} // namespace planning
} // namespace avt_341
