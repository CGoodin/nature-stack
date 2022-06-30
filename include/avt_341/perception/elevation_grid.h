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
    float highest = std::numeric_limits<float>::lowest();
    float second_highest = std::numeric_limits<float>::lowest();
    float height = 0.0f;
    bool filled = false;
    //float slope_x = 0.0f;
    //float slope_y = 0.0f;
    float slope = 0.0f;
    bool obstacle = false;
    bool has_dilated = false;
    uint8_t dilated_val = 0;
    float terrain = 0.0f;
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

    bool has_segmentation() const { return has_segmentation_; }

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

    void SetPersistentObstacles(bool persist){ persistent_obstacles_ = persist; }

    void SetStitchPoints(bool stitch_points){ stitch_points_ = stitch_points; }

    void SetFilterHighest(bool filter_high){ filter_highest_ = filter_high; }

    void SetUseElevation(bool use_elevation){
        use_elevation_ = use_elevation;
    }

    void ClearGrid();

    void UseDilation(bool use_dil){
        dilate_ = use_dil;
    }

    avt_341::msg::OccupancyGrid GetGrid(bool row_major=false, bool is_segmentation=false);

    void SetCorner(float llx, float lly){
        llx_ = llx;
        lly_ = lly;
    }

    void SetDilation(bool grid_dilate, float grid_dilate_x, float grid_dilate_y, float grid_dilate_proportion){
        dilate_ = grid_dilate;
        grid_dilate_x_ = grid_dilate_x;
        grid_dilate_y_ = grid_dilate_y;
        grid_dilate_proportion_ = grid_dilate_proportion;
    }


  private:
    uint8_t GetGridCellValue(const Cell & cell) const;
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
    float grid_dilate_x_;
    float grid_dilate_y_;
    float grid_dilate_proportion_;
    bool use_elevation_;
    bool stitch_points_;
    bool persistent_obstacles_;
    bool filter_highest_;
    const uint8_t GRID_MAX_VALUE = 100;
    const float GRID_SLOPE_MULT = 50.0f;
    bool has_segmentation_ = false;
};

} // namespace perception
} // namespace avt_341