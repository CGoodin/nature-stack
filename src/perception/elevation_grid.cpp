#include "nature/perception/elevation_grid.h"
#include <iostream>
#include <math.h>

namespace nature{
namespace perception{

ElevationGrid::ElevationGrid(){
  width_ = 200.0f;
  height_ = 200.0f;
  llx_ = -100.0f;
  lly_ = -100.0f;
  res_ = 0.5f;
  ResizeGrid();
  thresh_ = 1.0f;
  dilate_ = false;
  grid_dilate_x_ = 2.0f;
  grid_dilate_y_ = 2.0f;
  grid_dilate_proportion_ = 0.8f;
  use_elevation_ = false;
  stitch_points_ = true;
  filter_highest_ = false;
  persistent_obstacles_ = false;
}
    
ElevationGrid::~ElevationGrid(){

}

void ElevationGrid::ResizeGrid(){
  nx_ = (int)ceil(width_/res_);
  ny_ = (int)ceil(height_/res_);
  //if (n_%2!=0) n_ = n_+1;
  cells_.clear();
  std::vector<Cell> row;
  row.resize(ny_);
  cells_.resize(nx_,row);
}

void ElevationGrid::ClearGrid(){
  Cell empty_cell;
 for (int i=0;i<(nx_);i++){
    for (int j=0;j<(ny_);j++){ 
      if (persistent_obstacles_){
        uint8_t cellval = GetGridCellValue(cells_[i][j]);
        if (cellval<=0){
          cells_[i][j] = empty_cell;  
        }
      }
      else{
        cells_[i][j] = empty_cell;
      }
    }
 }
}

std::vector<nature::msg::Point32> ElevationGrid::AddPoints(nature::msg::PointCloud &point_cloud){

  bool has_segmentation_local = !point_cloud.channels.empty() && point_cloud.channels[0].name == "segmentation";
  has_segmentation_ = has_segmentation_local || has_segmentation_;

  if (!stitch_points_)ClearGrid();
  // fill the cells with highest and lowest points
  for (int i=0;i<point_cloud.points.size();i++){
    if (!(point_cloud.points[i].x==0.0 && point_cloud.points[i].y==0.0)){
      int xi = (int)floor((point_cloud.points[i].x - llx_)/res_);
      int yi = (int)floor((point_cloud.points[i].y - lly_)/res_);
      if (xi>=0 && xi<nx_ && yi>=0 &&yi<ny_){

        uint8_t current_val = GetGridCellValue(cells_[xi][yi]);
        if (!(persistent_obstacles_ && current_val>0)) {

          float h = point_cloud.points[i].z;
          cells_[xi][yi].filled = true;
          if (filter_highest_){
            if (h > cells_[xi][yi].highest ){
              cells_[xi][yi].second_highest = cells_[xi][yi].highest;
              cells_[xi][yi].highest = h;
              cells_[xi][yi].high = cells_[xi][yi].second_highest;
            }
            else if (h  > cells_[xi][yi].second_highest){
              cells_[xi][yi].second_highest = h;
              cells_[xi][yi].high = cells_[xi][yi].second_highest;
            }
          }
          else{
            if (h > cells_[xi][yi].high ) cells_[xi][yi].high = h;
          }
          if (h < cells_[xi][yi].low ) cells_[xi][yi].low = h;

          if (has_segmentation_local){
            float terr_val = point_cloud.channels[0].values[i];
            cells_[xi][yi].terrain = fmax(cells_[xi][yi].terrain, terr_val);
          }

        }

      }
    }
  }

  std::vector<int> cells_to_dilate_x {};
  std::vector<int> cells_to_dilate_y {};

  //find the slopes
  for (int i=0; i<nx_;i++){
    for (int j=0; j<ny_; j++){
      if (cells_[i][j].filled){
        cells_[i][j].height = cells_[i][j].high - cells_[i][j].low;
        //if (cells_[i][j].height/res_ > thresh_) cells_[i][j].obstacle = true;
        cells_[i][j].slope = cells_[i][j].height/res_;
        if(!cells_[i][j].has_dilated && cells_[i][j].slope > thresh_){
          cells_[i][j].has_dilated = true;
          cells_to_dilate_x.push_back(i);
          cells_to_dilate_y.push_back(j);
        }
      } // if cell filled
    } //over j
  } //over i

  //dilate the grid
  if(dilate_){
    int dsize_x = lround(grid_dilate_x_/res_);
    int dsize_y = lround(grid_dilate_y_/res_);

    for(const int & i : cells_to_dilate_x){
      if(i < dsize_x || i >= nx_-dsize_x){
        continue;
      }

      for(const int & j : cells_to_dilate_y){
        if(j < dsize_y || j >= ny_-dsize_y){
          continue;
        }

        if(!cells_[i][j].has_dilated){
          continue;
        }
        uint8_t grid_val = (uint8_t) (grid_dilate_proportion_ * GetGridCellValue( cells_[i][j]));
        for (int id=-dsize_x; id<=dsize_x; id++){
          for (int jd=-dsize_y; jd<=dsize_y; jd++){
            Cell & cell = cells_[i + id][j + jd];
            cell.dilated_val = std::max(grid_val, cell.dilated_val);
          }
        }
      }
    }
  }


  //loop back through the points and remove ground points
  std::vector<nature::msg::Point32> points;
  std::vector<nature::msg::Point32> surface_points;
  float hscale = 0.2f;
  for (int i=0;i<point_cloud.points.size();i++){
    if (!(point_cloud.points[i].x==0.0 && point_cloud.points[i].y==0.0)){
      int xi = (int)floor((point_cloud.points[i].x - llx_)/res_);
      int yi = (int)floor((point_cloud.points[i].y - lly_)/res_); 
      if (xi>=0 && xi<nx_ && yi>=0 &&yi<ny_){

        if (cells_[xi][yi].obstacle){
          if (point_cloud.points[i].z>(cells_[xi][yi].low + hscale*cells_[xi][yi].height)){
            points.push_back(point_cloud.points[i]);
          }
          else{
            surface_points.push_back(point_cloud.points[i]);
          }
        }
        else{
          surface_points.push_back(point_cloud.points[i]);
        }
      }
    }
  }
  point_cloud.points = points;
  return surface_points;
} // method AddPoints

uint8_t ElevationGrid::GetGridCellValue(const Cell & cell) const{
  if(!cell.filled)
    return 0;

  if(use_elevation_ && cell.high > thresh_)
    return GRID_MAX_VALUE;

  if(!use_elevation_){
    uint8_t val = (uint8_t) (GRID_SLOPE_MULT*cell.slope);
    if (val<0) val = 0;
    if (val>GRID_MAX_VALUE) val = GRID_MAX_VALUE;
    if (cell.slope>thresh_){
      return val;
    }
  }
  return 0;
}

nature::msg::OccupancyGrid ElevationGrid::GetGrid(bool row_major, bool is_segmentation){
  nature::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = res_;
  grid.info.width = nx_;
  grid.info.height = ny_;
  grid.info.origin.position.x = llx_;
  grid.info.origin.position.y = lly_;
  grid.info.origin.orientation.w = 1.0;
  grid.info.origin.orientation.x = 0.0;
  grid.info.origin.orientation.y = 0.0;
  grid.info.origin.orientation.z = 0.0;
  grid.data.resize(nx_*ny_);
  int c = 0;

  if(row_major){
    for (int j=0;j<ny_;j++){
      for (int i=0;i<nx_;i++){
        grid.data[c++] = is_segmentation ? (uint8_t)(cells_[i][j].terrain) : std::max(GetGridCellValue(cells_[i][j]), cells_[i][j].dilated_val);
      }
    }
  }else{
    for (int i=0;i<nx_;i++){
      for (int j=0;j<ny_;j++){
        grid.data[c++] = is_segmentation ? (uint8_t)(cells_[i][j].terrain) : std::max(GetGridCellValue(cells_[i][j]), cells_[i][j].dilated_val);
      }
    }
  }
  return grid;
}

} // namespace perception
} //namespace nature