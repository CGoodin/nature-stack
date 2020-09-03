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
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/OccupancyGrid.h"

namespace avt_341{
namespace perception{

struct Cell{
    float low = std::numeric_limits<float>::max();
    float high = std::numeric_limits<float>::lowest();
    float height = 0.0f;
    bool filled = false;
    float slope_x = 0.0f;
    float slope_y = 0.0f;
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
    std::vector<geometry_msgs::Point32> AddPoints(sensor_msgs::PointCloud &point_cloud);

    void SetSize(float s){
        size_ = s;
        cmin_ = -0.5f*s;
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

    nav_msgs::OccupancyGrid GetGrid(std::string grid_type);

  private:
    void ResizeGrid();
    void FillImage();
    std::vector< std::vector<Cell> > cells_;
    float size_;
    float res_;
    float thresh_;
    float cmin_;
    int n_;
    int n_2_;
    bool first_display_;
    bool dilate_;
};

} // namespace perception
} // namespace avt_341