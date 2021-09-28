#include "avt_341/perception/elevation_grid.h"
#include <iostream>
#include <math.h>

namespace avt_341{
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
      cells_[i][j] = empty_cell;
    }
 }
}

std::vector<avt_341::msg::Point32> ElevationGrid::AddPoints(avt_341::msg::PointCloud &point_cloud){
  // fill the cells with highest and lowest points
  for (int i=0;i<point_cloud.points.size();i++){
    if (!(point_cloud.points[i].x==0.0 && point_cloud.points[i].y==0.0)){
      int xi = (int)floor((point_cloud.points[i].x - llx_)/res_);
      int yi = (int)floor((point_cloud.points[i].y - lly_)/res_);
      if (xi>=0 && xi<nx_ && yi>=0 &&yi<ny_){
        float h = point_cloud.points[i].z;
        cells_[xi][yi].filled = true;
        if (h > cells_[xi][yi].high ) cells_[xi][yi].high = h;
        if (h < cells_[xi][yi].low ) cells_[xi][yi].low = h;
      }
    }
  }

  //find the slopes
  //for (int i=1;i<(nx_-1);i++){
  //  for (int j=1;j<(ny_-1);j++){
  for (int i=0; i<nx_;i++){
    for (int j=0; j<ny_; j++){
      if (cells_[i][j].filled){
        cells_[i][j].height = cells_[i][j].high - cells_[i][j].low;
        //if (cells_[i][j].height/res_ > thresh_) cells_[i][j].obstacle = true;
        cells_[i][j].slope = cells_[i][j].height/res_;
          /*
          if (cells_[i+1][j].filled){
            int ii = i+1;
            float run = (float)(ii-i)*res_;
            cells_[i][j].slope_x = (cells_[ii][j].high - cells_[i][j].high)/run;
            if (cells_[i][j].slope_x>thresh_){
              cells_[ii][j].obstacle = true;
            }
            else if (cells_[i][j].slope_x<-thresh_){
              cells_[i][j].obstacle = true;
            }
          }
          if (cells_[i-1][j].filled){
            int ii = i-1;
            float run = (float)(ii-i)*res_;
            cells_[i][j].slope_x = (cells_[i][j].high - cells_[ii][j].high)/run;
            if (cells_[i][j].slope_x>thresh_){
              cells_[ii][j].obstacle = true;
            }
            else if (cells_[i][j].slope_x<-thresh_){
              cells_[i][j].obstacle = true;
            }
          }

          if (cells_[i][j+1].filled){
            int jj = j+1;;
            float run = (float)(jj-j)*res_;
            cells_[i][j].slope_y = (cells_[i][jj].high - cells_[i][j].high)/run;
            if (cells_[i][j].slope_y>thresh_){
              cells_[i][jj].obstacle = true;
            }
            else if (cells_[i][j].slope_y<-thresh_){
              cells_[i][j].obstacle = true;
            }
          }
          if (cells_[i][j-1].filled){
            int jj = j-1;
            float run = (float)(jj-j)*res_;
            cells_[i][j].slope_y = (cells_[i][j].high - cells_[i][jj].high)/run;
            if (cells_[i][j].slope_y>thresh_){
              cells_[i][jj].obstacle = true;
            }
            else if (cells_[i][j].slope_y<-thresh_){
              cells_[i][j].obstacle = true;
            }
          }
        */
      } // if cell filled
    } //over j
  } //over i

  //dilate the grid
  if (dilate_){
    std::vector< std::vector<Cell> > dilated_cells = cells_;
    int dsize = 1;
    for (int i=dsize; i<(nx_-dsize);i++){
      for (int j=dsize; j<(ny_-dsize);j++){
        for (int id=-dsize; id<=dsize; id++){
          for (int jd=-dsize; jd<=dsize; jd++){
            if (cells_[i+id][j+jd].obstacle){
              dilated_cells[i][j].obstacle = true;
            }
          }
        }
      }
    }
    cells_ = dilated_cells;
  }
  //loop back through the points and remove ground points
  std::vector<avt_341::msg::Point32> points;
  std::vector<avt_341::msg::Point32> surface_points;
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

void ElevationGrid::GetGridCell(const std::string & grid_type, avt_341::msg::OccupancyGrid & grid, const int & i, const int & j, int & c){
  if (grid_type == "elevation"){
    if (cells_[i][j].filled){
      /*float val = cells_[i][j].high*30.0;
      if (val<0.0f)val = 0.0f;
      if (val>100.0f)val =100.0f;
      grid.data[c] = (uint8_t)(val);*/
      if (cells_[i][j].high>thresh_){
        grid.data[c] = 100;
      }
    }
  }
  else {
    if (cells_[i][j].filled){
      float s = cells_[i][j].slope; //sqrt(cells_[i][j].slope_x*cells_[i][j].slope_x + cells_[i][j].slope_y*cells_[i][j].slope_y);
      uint8_t val = (uint8_t) (50.0f*s);
      if (val<0) val = 0;
      if (val>100) val = 100;
      if (s>thresh_){
        grid.data[c] = val;
      }
      else{
        grid.data[c] = 0;
      }
    }
    else{
      grid.data[c] = 0;
    }
  }
  c++;
}

avt_341::msg::OccupancyGrid ElevationGrid::GetGrid(std::string grid_type, bool row_major){
  avt_341::msg::OccupancyGrid grid;
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
        GetGridCell(grid_type, grid, i, j, c);
      }
    }
  }else{
    for (int i=0;i<nx_;i++){
      for (int j=0;j<ny_;j++){
        GetGridCell(grid_type, grid, i, j, c);
      }
    }
  }
  return grid;
}

} // namespace perception
} //namespace avt_341