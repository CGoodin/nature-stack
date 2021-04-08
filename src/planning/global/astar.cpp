
#include <queue>
#include <limits>
#include <cmath>
#include <fstream>
#include <iostream>
#include <algorithm>
// project includes
#include "avt_341/planning/global/astar.h"
#include "avt_341/planning/global/astar_cell.h"

namespace avt_341 {
namespace planning{

Astar::Astar(){
  dfac_ = 0;
}

Astar::~Astar(){

}

std::vector<int> Astar::FoldIndex(int n){
  std::vector<int> c;
	c.resize(2);
  c[0] = n%width_;
  c[1] = (int)floor( (1.0*n)/(1.0*width_) );
  return c;
}

void Astar::AllocateMap(int h, int w, int init_val){
  height_ = h;
  width_ = w;
  weights_.clear();
  weights_.resize(height_ * width_, init_val);
  paths_.clear();
  paths_.resize(height_ * width_, -1);
  for (int i = 0; i < width_; i++){
    for (int j = 0; j < height_; j++){
      if (i == 0 || j == 0 ||
          i == (width_ - 1) ||
          j == (height_ - 1)){
        int n = FlattenIndex(i, j);
        weights_[n] = 0;
      }
    }
  }

	std::vector<float> column;
	column.resize(height_, 0.0f);
	map_.resize(width_, column);
}

void Astar::SetMapValue(int i, int j, int val){
  weights_[FlattenIndex(i,j)]=val;
	map_[i][j] = (float)val;
}

bool Astar::LineOfSight(std::vector<int> p0, std::vector<int> p1){
  if (p0.size()!=2 || p1.size()!=2) return false;
  // see: https://news.movel.ai/theta-star/
  int x0 = p0[0];
  int y0 = p0[1];
  int x1 = p1[0];
  int y1 = p1[1];

  int dy = y1-y0;
  int dx = x1-x0;

  int f = 0;

  int sx = p0[0];
  int sy = p0[1];
  if (dy < 0){
    dy = -dy;
    sy = -1;
  }
  else{
    sy = 1;
  }
      
  if (dx < 0) {
    dx = -dx;
    sx = -1;
  }
  else{
    sx = 1;
  }

  if (dx >= dy){
    while (x0 != x1){
      f = f + dy;
      if (f >= dx ){
        if (map_[x0+((sx-1)/2)][y0 + ((sy-1)/2)]>0 ){
          return false;
        }
        y0 = y0 + sy;
        f = f - dx;
      }
      if (f != 0 && map_[x0+((sx-1)/2)][y0 + ((sy-1.0f)/2.0f)]>0) {
        return false;
      }
      if (dy== 0 && map_[x0 + ((sx-1)/2)][y0]>0 && map_[x0 + ((sx-1)/2)][ y0-1]>0){
        return false;
      }
      x0 = x0 + sx;
    }
  }  
  else{
    while (y0 != y1){
      f = f + dx;
      if (f >= dy){
          if ( map_[x0 + ((sx - 1)/2)][ y0 + ((sy-1)/2)]>0 ){
              return false;
          }
          x0 = x0 + sx;
          f = f - dy;
      }
      if (f!=0 && map_[x0+((sx-1)/2)][ y0 + ((sy-1)/2)]>0 ){
        return false;
      }
      if (dx==0 && map_[x0][y0+((sy-1)/2)]>0 && map_[x0-1 ][ y0 + ((sy-1)/2)]>0 ){
        return false;
      }
      y0 = y0 + sy;
    }
  }
  
  return true; 
}

// For path of length n
void Astar::PostSmoothing(){
  if (path_.size()>2){
    int k = 0;
    std::vector<std::vector<int> > t; 
    t.push_back(path_[0]);
    for (int i =1;i<path_.size()-1;i++){
      if (!LineOfSight(t[k], path_[i+1])){
          k++;
          t.push_back(path_[i]);
      }
    }
    t.push_back(path_.back());
    path_ = t;
  }
}

// manhattan distance: requires each move to cost >= 1
float Astar::Heuristic(int i0, int j0, int i1, int j1) {
  //straight line distance
  int x = i1-i0;
  int y = j1-j0;
  return (float)sqrt(x*x+y*y);

  //return std::max(std::abs(x),std::abs(y));
  //manhattan distance
  //return std::abs(i0 - i1) + std::abs(j0 - j1);
  // diagonal heuristic
  //float D2 = 1.0f;
  //float D = 1.414214f; //1.0f;
  //float dx = abs(i1-i0);
  //float dy = abs(j1-j0);
  //return D * (dx + dy) + (D2 - 2.0f * D) * std::min(dx, dy);
}

bool Astar::Solve() {
  std::fill(paths_.begin(), paths_.end(),-1);
  
  const float INF = std::numeric_limits<float>::infinity();

  AStarCell start_node(start_, 0.);
  AStarCell goal_node(goal_, 0.);

  float* costs = new float[height_ * width_];
  for (int i = 0; i < height_ * width_; ++i)
    costs[i] = INF;
  costs[start_] = 0.;

  std::priority_queue<AStarCell> nodes_to_visit;
  nodes_to_visit.push(start_node);

  int* nbrs = new int[4];

  bool solution_found = false;
  while (!nodes_to_visit.empty()) {
    // .top() doesn't actually remove the node
    AStarCell cur = nodes_to_visit.top();

    if (cur == goal_node) {
      solution_found = true;
      break;
    }

    nodes_to_visit.pop();

    // check bounds and find up to four neighbors
    nbrs[0] = (cur.idx / width_ > 0) ? (cur.idx - width_) : -1;
    nbrs[1] = (cur.idx % width_ > 0) ? (cur.idx - 1) : -1;
    nbrs[2] = (cur.idx / width_ + 1 < height_) ? (cur.idx + width_) : -1;
    nbrs[3] = (cur.idx % width_ + 1 < width_) ? (cur.idx + 1) : -1;
    for (int i = 0; i < 4; ++i) {
      if (nbrs[i] >= 0) {
        // the sum of the cost so far and the cost of this move
        float new_cost = costs[cur.idx] + weights_[nbrs[i]];
        if (new_cost < costs[nbrs[i]]) {
          costs[nbrs[i]] = new_cost;
          float priority = new_cost + Heuristic(nbrs[i] / width_,
                                                nbrs[i] % width_,
                                                goal_ / width_,
                                                goal_ % width_);
          // paths with lower expected cost are explored first
          nodes_to_visit.push(AStarCell(nbrs[i], priority));
          paths_[nbrs[i]] = cur.idx;
        }
      }
    }
  }

  delete[] costs;
  delete[] nbrs;

  if (solution_found)ExtractPath();
  
  return solution_found;
}

void Astar::ExtractPath(){
  path_.clear();
	path_world_.clear();
  int path_idx = goal_;
	std::vector<float> point;
	point.resize(2);
  while (path_idx!=start_){
    std::vector<int> c = FoldIndex(path_idx);
    path_.push_back(c);
    path_idx = paths_[path_idx];
  }
	
  //smooth path out
  PostSmoothing();

  // put the smoothed path in world coordinates
  std::vector<std::vector<float> > smoothed_path;
  for (int i=0;i<path_.size();i++){
		point = IndexToPoint(path_[i]);
		smoothed_path.push_back(point);
  }
  
  // the smoothed path may have much fewer points. Fill in the missing parts
  std::vector<std::vector<float> > filled_in_path_world;
  for (int i=0;i<smoothed_path.size()-1;i++){
    float px = smoothed_path[i][0];
    float py = smoothed_path[i][1];
    float dx = smoothed_path[i+1][0]-px;
    float dy = smoothed_path[i+1][1]-py;
    double d = sqrt(dx*dx + dy*dy);
    double ltx = map_res_*dx/d;
    double lty = map_res_*dy/d;
    while (d>2.0*map_res_){
      std::vector<float> point;
      point.push_back(px);
      point.push_back(py);
      filled_in_path_world.push_back(point);
      px += ltx;
      py += lty;
      dx = smoothed_path[i+1][0]-px;
      dy = smoothed_path[i+1][1]-py;
      d = sqrt(dx*dx + dy*dy);
    }
  }
  path_world_ = filled_in_path_world;

  std::reverse(path_.begin(), path_.end());
	std::reverse(path_world_.begin(), path_world_.end());

}

void Astar::SaveMap(std::string imname){
	cimg_library::CImg<float> img(disp_);
	img.save(imname.c_str());
}

void Astar::Display(){
  if (map_.size()==0) return;
  if (map_[0].size()==0)return;
  int nx = map_.size();
  int ny = map_[0].size();
  cimg_library::CImg<float> img;
	img.assign(nx,ny,1,3,0.0f);
  float yellow[3], green[3], red[3];
	yellow[0] = 255.0f; yellow[1]=255.0f; yellow[2]=0.0f;
	green[0] = 0.0f; green[1]=255.0f; green[2]=0.0f;
	red[0]=255.0f; red[1]=0.0f; red[2]=0.0f;
  for (int i=0;i<nx;i++){
    for (int j=0;j<ny;j++){
      if (map_[i][j]>0) img.draw_point(i,j,red);
    }
  }

	for (int i=0;i<path_world_.size();i++){
		int ix = (int)floor((path_world_[i][0] - llx_) / map_res_);
		int iy = (int)floor((path_world_[i][1] - lly_) / map_res_);
		img.draw_point(ix,iy,(float *)&yellow);
	}

  std::vector<float> goal = GetCurrentGoal(); 

	int gx = (int)floor((goal[0] - llx_) / map_res_);
	int gy = (int)floor((goal[1] - lly_) / map_res_);
	img.draw_circle(gx,gy,2,(float *)&green);

	img.mirror('y');

  disp_ = img;
}

std::vector<std::vector<float> > Astar::PlanPath(nav_msgs::OccupancyGrid *grid, std::vector<float> goal, std::vector<float> position) {
	if (grid->info.height<=0 || grid->info.width<=0) return path_world_;

  SetCornerCoords(grid->info.origin.position.x, grid->info.origin.position.y);
	SetMapRes(grid->info.resolution);

	std::vector<int> gi = PointToIndex(goal[0], goal[1]);
	std::vector<int> si = PointToIndex(position[0], position[1]);

  if (si[0]<0)si[0]=0;
  if (si[0]>=grid->info.width) si[0] = grid->info.width-1;
  if (si[1]<0)si[1]=0;
  if (si[1]>=grid->info.height) si[1] = grid->info.height-1;
  if (gi[0]<0)gi[0]=0;
  if (gi[0]>=grid->info.width) gi[0] = grid->info.width-1;
  if (gi[1]<0)gi[1]=0;
  if (gi[1]>=grid->info.height) gi[1] = grid->info.height-1;

	AllocateMap(grid->info.height, grid->info.width, 0);
	SetGoal(gi[0], gi[1]);
	SetStart(si[0], si[1]);
	std::vector<float> gr;
	gr = GetCurrentGoal();
  int n =0;
	for (int i=0;i<width_;i++){
		for (int j = 0; j < height_; j++) {
			SetMapValue(i, j, grid->data[n]);
      n++;
		}
	}

  // dilate
  if (dfac_>0){
    std::vector<std::vector<float> > dmap = map_;
    for (int i=dfac_;i<width_-dfac_;i++){
      for (int j = dfac_; j < height_-dfac_; j++) {
        int val = 0;
        for (int ii=i-dfac_;ii<=i+dfac_;ii++){
          for (int jj=j-dfac_;jj<=j+dfac_;jj++){
            if (map_[ii][jj]>0) val = 100;
          }
        }
        dmap[i][j] = val;
      }
    }
    for (int i=1;i<width_-1;i++){
      for (int j = 1; j < height_-1; j++) {
        SetMapValue(i,j,dmap[i][j]);
      }
    }
  }
  

	bool solved = Solve();
	if (!solved) {
		std::cerr << "WARNING: A* failed to solve map " << std::endl;
	}



  return path_world_;
}
} // namespace planning
} // namespace avt_341