#include "avt_341/planning/local/pf_planner.h"
#include <algorithm>

namespace avt_341 {
namespace planning{
PfPlanner::PfPlanner() {
	kp_ = 5.0f;
	eta_ = 100.0f;
	obs_cutoff_dist_ = 20.0f;
}

avt_341::msg::Path PfPlanner::Plan(avt_341::msg::OccupancyGrid grid, avt_341::msg::Odometry odom){
	
	float sx = odom.pose.pose.position.x;
	float sy = odom.pose.pose.position.y;
	float gx = goal_x_; 
	float gy = goal_y_;

	// first populate the obstacle list from the current grid
	std::vector<float> ox, oy;
	int ndx = 0;
	for (int i=0;i<grid.info.width;i++){
    float x = grid.info.origin.position.x + (i+0.5f)*grid.info.resolution;
    float dx = sx-x;
    for (int j=0;j<grid.info.height;j++){
      float y = grid.info.origin.position.y + (j+0.5f)*grid.info.resolution;
      float dy = sy-y;
      if (grid.data[ndx]>0){
        float d = sqrtf(dx*dx+dy*dy);
          if (d<obs_cutoff_dist_){
            ox.push_back(x);
            oy.push_back(y);
					} // if closer than cutoff distance
			} // if cell occupied
		} // loop over j
	} // loop over i

	// run the potential field algorithm
	PotentialFieldPlanning(grid.info.origin.position.x,  grid.info.origin.position.y, grid.info.resolution, sx, sy, gx, gy, ox, oy);

	// copy the result to a path message
	avt_341::msg::Path path;
  for (int i=0;i<(int)rx_.size();i++){
		avt_341::msg::PoseStamped pose;
    pose.pose.position.x = rx_[i]; 
    pose.pose.position.y = ry_[i];
    path.poses.push_back(pose);
	}
	
	return path;
}


void PfPlanner::SetGoal(float gx, float gy){
	goal_x_ = gx;
	goal_y_ = gy;
}

float PfPlanner::Hypot(float x, float y){
  return sqrtf(x*x+y*y);
}
    
float PfPlanner::CalcAttractivePotential(float x, float y, float gx, float gy){
	return 0.5f * kp_ * Hypot(x - gx, y - gy);
}

float PfPlanner::CalcRepulsivePotential(float x, float y, std::vector<float> ox, std::vector<float> oy){

	float repul = 0.0f;

	if (ox.size()!=oy.size())return repul;

	for (int i=0;i<(int)ox.size();i++){
		float d = Hypot(x - ox[i], y - oy[i]);
		repul += eta_/(d*d+0.1f);
	}
	return repul;
}

std::vector<std::vector<float> > PfPlanner::GetMotionModel(float step){
	std::vector<std::vector<float> > motion;
	std::vector<float> col;
	col.resize(2,0.0f);
	motion.resize(8,col);
	motion[0][0] = step;
	motion[1][1] = step;
	motion[2][0] = -step;
	motion[3][1] = step;
	motion[4][0] = -step;
	motion[4][1] = -step;
	motion[5][0] = -step;
	motion[5][1] = step;
	motion[6][0] = step;
	motion[6][1] = -step;
	motion[7][0] = step;
	motion[7][1] = step;
	return motion;
}

void PfPlanner::PotentialFieldPlanning(float minx, float miny, float reso, float sx, float sy, float gx, float gy, std::vector<float> ox, std::vector<float> oy){

	rx_.clear();
	ry_.clear();
	
	//# search path
	//float minx = grid.info.origin.position.x;
	//float miny = grid.info.origin.position.y;
	//float reso = grid.info.resolution;
	float d = Hypot(sx - gx, sy - gy);
	float max_steps = floor(3.0f*d);
	int ix = (int)floor((sx - minx) / reso);
	int iy = (int)floor((sy - miny) / reso);
	int gix = (int)floor((gx - minx) / reso);
	int giy = (int)floor((gy - miny) / reso);
	rx_.push_back(sx);
	ry_.push_back(sy);
	float xp = sx;
	float yp = sy;
	std::vector<std::vector<float > > motion = GetMotionModel(reso);
	int nsteps = 0;
	
	while (d >= reso && nsteps<max_steps){
		
		float minp = std::numeric_limits<float>::max(); //  float("inf")
		float mindx = reso;
		float mindy = 0.0f; 
		
		for (int i=0;i<(int)motion.size();i++){
			float dx = reso*motion[i][0];
			float dy = reso*motion[i][1];
			float xx = xp+dx;
			float yy = yp+dy;
			float ug = CalcAttractivePotential(xx, yy, gx, gy);
			float uo = CalcRepulsivePotential(xx, yy, ox, oy);
			float p = ug + uo;
			if (minp > p){
				minp = p;
				mindx = dx;
				mindy = dy;
			}
		}
		xp = xp+mindx;
		yp = yp+mindy;
		d = Hypot(gx - xp, gy - yp);
		rx_.push_back(xp);
		ry_.push_back(yp);
		nsteps = nsteps+1;
	}
}

} // namespace planning
} // namespace avt_341