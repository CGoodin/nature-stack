/**
 * \class ElevationGrid
 *
 * A slope-based obstacle detection algorithm. 
 * The world is divided into 2D cells. 
 * The highest and lowest point are used to calculate the slope in each cell.
 * Cells that exceed a slope threshold are flagged as obstcles.
 *
 * \author Chris Goodin
 *
 * \date 9/3/2020
 */
#include <vector>
#include <limits>
#include <string>
#include "avt_341/node/ros_types.h"

namespace avt_341{
namespace perception{

struct Cell{
    float low = std::numeric_limits<float>::max();
    float high = std::numeric_limits<float>::lowest();
    float height = 0.0f;
    bool filled = false;
    //float slope_x = 0.0f;
    //float slope_y = 0.0f;
    float slope = 0.0f;
    bool obstacle = false;
};

class ElevationGrid{
  public:
    ElevationGrid();

    ~ElevationGrid();

    /**
     * Add points to be processed 
     * Modifies the input to be only obstacle points
     * Returns surface points
     * \param point_cloud PointCloud message
     */
    std::vector<avt_341::msg::Point32> AddPoints(avt_341::msg::PointCloud &point_cloud);

    void SetSize(float s){
        width_ = s;
        height_ = s;
        ResizeGrid();
    }

    void SetSize(float width,float height){
        width_ = width;
        height_ = height;
        ResizeGrid();
    }

    void SetRes(float r){
        res_ = r;
        ResizeGrid();
    }

    void SetSlopeThreshold(float tr){
        thresh_ = tr;
    }

    void ClearGrid();

    void UseDilation(bool use_dil){
        dilate_ = use_dil;
    }

    avt_341::msg::OccupancyGrid GetGrid(std::string grid_type, bool row_major=false);

    void SetCorner(float llx, float lly){
        llx_ = llx;
        lly_ = lly;
    }

  private:
    void GetGridCell(const std::string & grid_type, avt_341::msg::OccupancyGrid & grid, const int & i, const int & j, int & c);
    void ResizeGrid();
    void FillImage();
    std::vector< std::vector<Cell> > cells_;
    float width_;
    float height_;
    float res_;
    float thresh_;
    int nx_,ny_;
    bool first_display_;
    bool dilate_;
    float llx_;
    float lly_;
};

} // namespace perception
} // namespace avt_341