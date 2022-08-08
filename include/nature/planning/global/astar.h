#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <nature/visualization/base_visualizer.h>
#include "nature/node/ros_types.h"

namespace nature {
namespace planning{

/**
 * Astar map class with solve functions and map accessor methods.
 * The map holds obstacle values from 0 to 100. 0=no obstacle, 100=impassable.
 */
class Astar {
 public:
  /// Constructor
  Astar(std::shared_ptr<nature::visualization::VisualizerBase> visualizer);

  /// Destructor
  ~Astar();

  /// Inherited from base class
  void Display();

  /// Inherited from base class
  void SaveMap(std::string ofname);
  
	/// Inherited from base class, return path in world coordinates
	std::vector<std::vector<float> > *GetCurrentPath() { return &path_world_; }

	/// Inherited from base class, return the current map
	std::vector<std::vector<float> > *GetCurrentMap() { return &map_; }

	/// Inherited from base class, return goal in world coordinates
	std::vector<float> GetCurrentGoal() {
		std::vector<int> gi = FoldIndex(goal_);
		std::vector<float> goal_world = IndexToPoint(gi);
		return goal_world;
	}

	/// Inherited from planner base class.
	float GetXMin() { return llx_; }

	/// Inherited from planner base class.
	float GetYMin() { return lly_; }

	/// Inherited from planner base class.
	float GetGridResolution() { return map_res_; }

	/// Inherited from planner base class. Returns the number of horizontal cells.
	int GetGridWidth() { return width_; }

	/// Inherited from planner base class. Returns the number of vertical cells.
	int GetGridHeight() { return height_; }

	/// Inherited from planner base class.
	std::vector<std::vector<float> > PlanPath(nature::msg::OccupancyGrid *grid, nature::msg::OccupancyGrid *segmentation_grid, std::vector<float> goal, std::vector<float> position);

  /**
   * Allocate memory for the map and initialize
   * \param height Height of the map, in cells
   * \param width Width of the map, in cells
   * \param init_val Initial value for all cells, from 0 to 100
   */
  void AllocateMap(int height, int width, int init_val);

  /**
   * Set the value of cell (i,j).
   * \param i Vertical index of the cell to set
   * \param j Horizontal index of the cell to set
   * \param val_height Value to set, [0,100]
   */
  void SetMapValue(int i, int j, int val_height, int val_seg);

  /**
   * Returns the map value of cell (i,j).
   * \param i Vertical index of cell to get
   * \param j Horizontal index of cell to get
   */
  int GetMapValue(int i, int j){return weights_[FlattenIndex(i,j)];}

  /**
   * Sets cell (i,j) as the goal point.
   * \param i Vertical index of goal cell
   * \param j Horizontal index of goal cell
   */
  void SetGoal(int i, int j){goal_ = FlattenIndex(i,j);}

  /**
   * Sets cell (i,j) as the current location of the vehicle
   * \param i Vertical index of the vehicle location
   * \param j Horizontal index of the vehicle location
   */
  void SetStart(int i, int j){start_ = FlattenIndex(i,j);}

  /**
   * Solve the A* map. Returns true if a path was found.
   */
  bool Solve();
 
  /// Return a list of indices specifying the current path.
  std::vector<std::vector<int> > GetPath(){return path_;}

  /**
   * Set the ENU coordinates of the bottom left corner
   * \param x The East ENU coordinate
   * \param y The North ENU coordinate
   */
  void SetCornerCoords(float x, float y){
    llx_ = x;
    lly_ = y;
  }

  /**
   * Set the resolution of the map cells in meters
   * \param res The resolution in meters 
   */
  void SetMapRes(float res){
    map_res_ = res;
  }

  /// Get the resolution of the map in meters
  float GetRes(){return map_res_;}
  
  /**
   * Convert a point in global ENU to map coordinates
   * \param x The East ENU coordiante
   * \param y The North ENU coordinate 
   */
  std::vector<int> PointToIndex(float x, float y){
    std::vector<int> c;
		c.resize(2);
    c[0] = (int)((x-llx_)/map_res_);
    c[1] = (int)((y-lly_)/map_res_);
    return c;
  }

  /**
   * Convert a point in map coordinates to global ENU
   * \param c The index of the map cell 
   */
  std::vector<float> IndexToPoint(std::vector<int> c){
    std::vector<float> p;
    p.resize(2);
    p[0] = (c[0]+0.5f)*map_res_ + llx_;
    p[1] = (c[1]+0.5f)*map_res_ + lly_;
    return p;
  }

    /**
   * Determine if a point is in the map. 
   * Return false in the point is not in the map
   * \param c The index of the map cell 
   */
  bool IsInMap(std::vector<int> c){
    bool isin = false;
    if (c[0]>=0 && c[0]<width_ && c[1]>=0 && c[1]<height_){
      isin = true;
    }
    return isin;
  }
  
  /**
   * Set the factor by which to dilate the map
   * \param dfac The dilation factor 
   */
  void SetDilationFactor(int dfac){dfac_ = dfac;}

 private:
  std::vector<int> FoldIndex(int n);
  
  int FlattenIndex(int i, int j){return j*width_+i;}
  
  /// Heuristic
  float Heuristic(int i0, int j0, int i1, int j1);

  /// Flattened occupancy grid
  std::vector<int> weights_;

	/// unflattened occupancy grid
	std::vector<std::vector<float> > map_;


  ///height of the grid
  int height_;

  ///width of the grid
  int width_;

  ///flattened index of the goal point
  int goal_;

  ///flattened index of the start point
  int start_;

  /// map dilation factor
  int dfac_;

  /// calculated path
  std::vector<int> paths_;

  //std::vector<MapIndex> path_;
	std::vector<std::vector<int> > path_;
	std::vector<std::vector<float> > path_world_;

  bool ExtractPath();
  void PostSmoothing();
  bool LineOfSight(std::vector<int> p0, std::vector<int> p1);

  float llx_,lly_;
  float map_res_;

  std::shared_ptr<nature::visualization::VisualizerBase> visualizer_;
};

} // namespace planning
} // namespace nature

#endif
