/*
Non-Commercial License - Mississippi State University Off-Road Traversability Algorithm

REPO: https://gitlab.com/cgoodin/off_road_traversability

CONTACT: cgoodin@cavs.msstate.edu

ACKNOWLEDGEMENT:
Mississippi State University, Center for Advanced Vehicular Systems (CAVS)

CITATION:
Goodin, C., Dabbiru, L., Hudson, C., Mason, G., Carruth, D., & Doude, M. (2021, April). 
Fast terrain traversability estimation with terrestrial lidar in off-road autonomous navigation. 
In Unmanned Systems Technology XXIII (Vol. 11758, p. 117580O). International Society for Optics and Photonics.

NOTICE:
Do not share or distribute. Software is authorized for use only by the approved recepient.

Copyright 2022 (C) Mississippi State University
*/
#ifndef VOXEL_GRID_H_
#define VOXEL_GRID_H_

// c++ includes
#include <limits>
#include <vector>
// ros includes
#include "nav_msgs/OccupancyGrid.h"
// project includes
#include "nature/perception/ftte/vehicle.h"
#include "nature/perception/ftte/traverse_cell.h"
#include "nature/perception/ftte/plane.h"
#include "nature/thirdparty/glm/glm.hpp"
#include "nature/thirdparty/CImg.h"


namespace traverselib {

class VoxelGrid {
public:

	VoxelGrid(){
		initialized_ = false;
	}

	VoxelGrid(glm::vec3 llc, glm::vec3 urc, float res);

	void Initialize(glm::vec3 llc, glm::vec3 urc, float res);

	void Move(glm::vec3 new_llc, glm::vec3 new_urc);

	void PlotGround(); 

	void SaveGroundPlot(std::string fname);

	void SaveSlicePlot(std::string fname);

	nav_msgs::OccupancyGrid GetTraversabilityAsOccupancyGrid(bool row_major);
	
	void WriteStats(std::string fname);

	glm::ivec3 GetDim() { return dim_; }

	void CalculateSlope();
	void CalculateRoughness();

	

	void PlotSlope();
	void PlotVegDensity();
	void PlotRoughness();
	void PlotTraversability();
	void PlotConfidence();
	void PlotRci();

	void PlotTravAndImage(cimg_library::CImg<float> image, std::string prefix);

	void SaveRciPlot(std::string fname);
	void SaveConfidencePlot(std::string fname);
	void SaveVegDensityPlot(std::string fname);
	void SaveTraversabilityPlot(std::string fname);
	void SaveSlopePlot(std::string fname);
	void SaveRoughPlot(std::string fname);

	void SetVehicle(traverselib::Vehicle vehicle) {
		vehicle_ = vehicle;
		veg_hi_lo_cutoff_.x = vehicle_.GetBumperHeight();
		veg_hi_lo_cutoff_.y = vehicle_.GetRoofHeight();
		vehicle_.SetGridResolution(res_);
	}

	float GetTraversabilityAtPoint(glm::vec3 p);

	float GetTraversabilityAtCell(int i, int j);

	float GetTraversabilityAtCell(glm::ivec3 idx) {
		return GetTraversabilityAtCell(idx.x, idx.y);
	}

	void AddRegisteredPoints(std::vector<glm::vec3> &points, glm::vec3 position);

	void CalculateLocalAvg();

	float GetDefaultTraversability() { return default_traversability_; }

	void SetDefaultTraversability(float dft) { default_traversability_ = dft; }

	void UsePlaneFitting(bool use_planes){ use_plane_fitting_ = use_planes; }

	void SetAveragingRadius(float ar){averaging_radius_ = ar;}

	traverselib::Vehicle *GetVehicle(){return &vehicle_;}

	void SetPrintTimingInfo(bool to_print){ print_timing_info_ = to_print; }

	bool Initialized(){ return initialized_; }

protected:

	bool LineLineIntersect(glm::vec2 origin, glm::vec2 endpoint, glm::vec2 line_p0, glm::vec2 line_p1);
	bool LineBoxIntersect(glm::vec2 origin, glm::vec2 endpoint, glm::ivec2 box_idx);
	bool ArePointsColinear(std::vector<glm::vec3> points);
	void RayMarching(glm::vec3 start_point, glm::vec3 end_point);
	void AddVehiclePathToImage(cimg_library::CImg<float> &img);
	void CalculateGround();
	
	// utility methods
	glm::ivec3 PointToIndex(glm::vec3 p);
	glm::ivec2 PointToIndex(glm::vec2 p);
	glm::vec3 IndexToPoint(glm::ivec3 idx);
	bool InGrid(glm::ivec3 idx);
	bool InGrid(glm::ivec2 idx);

	bool use_plane_fitting_;
	bool print_timing_info_;
	bool initialized_;

	// map data
	std::vector<std::vector<TraverseCell> > cells_;
	std::vector<std::vector<float> > ground_;
	std::vector<std::vector<float> > local_avg_;
	std::vector<std::vector<Plane> > fit_planes_;
	std::vector<std::vector<float> > slope_;
	std::vector<std::vector<float> > roughness_;
	std::vector<std::vector<float> > impermeability_; 
	std::vector<std::vector<float> > rci_;
	std::vector<glm::vec2> vehicle_path_;
	float max_slope_;
	float max_impermeability_; 
	float max_roughness_;
	float min_ground_;
	float max_ground_;
	float rci_max_;

	traverselib::Vehicle vehicle_;

	// parameters
	float res_;
	float res_squared_;
	glm::vec3 llc_;
	glm::vec3 urc_;

	glm::ivec3 dim_;

	float averaging_radius_;
	glm::vec2 veg_hi_lo_cutoff_;

	float default_traversability_;

	// plotting displays
	cimg_library::CImg<float> DrawSlope();
	cimg_library::CImg<float> DrawGround();
	cimg_library::CImg<float> DrawVegDensity();
	cimg_library::CImg<float> DrawRoughness();
	cimg_library::CImg<float> DrawTraversability();
	cimg_library::CImg<float> DrawConfidence();
	cimg_library::CImg<float> DrawRci();

	cimg_library::CImgDisplay ground_disp_;
	cimg_library::CImgDisplay slice_disp_;
	cimg_library::CImgDisplay slope_disp_;
	cimg_library::CImgDisplay rough_disp_;
	cimg_library::CImgDisplay veg_disp_;
	cimg_library::CImgDisplay rci_disp_;
	cimg_library::CImgDisplay trav_disp_;
	cimg_library::CImgDisplay conf_disp_;

	std::vector<glm::ivec2> path_taken_;
	int max_num_points_;
	float elev_avg_;

	bool local_avg_complete_;
};

} // namespace traverselib

#endif
