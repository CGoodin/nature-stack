// c++ includes
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
// ros includes
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/LinearMath/Transform.h"
// nature includes
#include "nature/perception/ftte/voxel_grid.h"

/// Convert any type to a string
template <class T>
inline std::string ToString(T x, int zero_padding) {
	std::stringstream ss;
	ss << std::setfill('0')<<std::setw(zero_padding)<<x;
	std::string str = ss.str();
	return str;
};

nav_msgs::Odometry current_pose;
bool odom_rcvd = false;
bool points_rcvd = false;
bool use_registered_points = false;
glm::vec3 current_position;
std::vector<glm::vec3> current_points;
bool using_loam = false;

void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& rcv_cloud){
    sensor_msgs::PointCloud point_cloud;
	bool converted = sensor_msgs::convertPointCloud2ToPointCloud(*rcv_cloud,point_cloud);
	if (odom_rcvd && converted){
		current_points.clear();
		if (use_registered_points){
			for (int p=0;p<point_cloud.points.size();p++){
				if (using_loam){
					current_points.push_back(glm::vec3(point_cloud.points[p].z, point_cloud.points[p].x,point_cloud.points[p].y));
				}
				else{
					current_points.push_back(glm::vec3(point_cloud.points[p].x, point_cloud.points[p].y,point_cloud.points[p].z));
				}
				
			}
		}
		else{
			tf::Quaternion q(current_pose.pose.pose.orientation.x, current_pose.pose.pose.orientation.y, current_pose.pose.pose.orientation.z, current_pose.pose.pose.orientation.w);
			tf::Matrix3x3 R(q);
			tf::Vector3 origin(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
			//std::vector<geometry_msgs::Point32> points;
			for (int p=0;p<point_cloud.points.size();p++){
				tf::Vector3 v;
				v = tf::Vector3(point_cloud.points[p].x, point_cloud.points[p].y,point_cloud.points[p].z);
				tf::Vector3 vp = (R*v) + origin;
				current_points.push_back(glm::vec3(vp.x(),vp.y(),vp.z()));
			}
		}
	}
	points_rcvd = true;
}

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& rcv_odom){
	current_pose = *rcv_odom;
	if (using_loam){
		current_pose.pose.pose.position.x = rcv_odom->pose.pose.position.z;
		current_pose.pose.pose.position.y = rcv_odom->pose.pose.position.x;
		current_pose.pose.pose.position.z = rcv_odom->pose.pose.position.y;
	}
	odom_rcvd = true;
	current_position = glm::vec3(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
}

//void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& rcv_pose){
//	current_pose.pose = rcv_pose->pose;
//	odom_rcvd = true;
//	current_position = glm::vec3(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
//}

int main(int argc, char *argv[]) {

	ros::init(argc, argv, "nature_ftte_node");
	
	ros::NodeHandle n;

	// subscribe to odometry and point cloud message
	ros::Subscriber pc_sub = n.subscribe("nature/points",10,PointCloudCallback);
	ros::Subscriber odom_sub = n.subscribe("nature/odometry",10, OdometryCallback);
	ros::Publisher grid_pub = n.advertise<nav_msgs::OccupancyGrid>("nature/occupancy_grid", 10);
	ros::Publisher grid_pub_vis = n.advertise<nav_msgs::OccupancyGrid>("nature/occupancy_grid_vis", 10);

	if (ros::param::has("~use_registered_points")){
    	ros::param::get("~use_registered_points",use_registered_points);
	}

	if (ros::param::has("~use_loam")){
    	ros::param::get("~use_loam",using_loam);
	}

	float map_width = 150.0f;
	float map_length = 150.0f;
	float map_res = 1.0f; 
	bool use_planes = false;
	bool show_timing = false;
	bool fixed_map = true;
	if (ros::param::has("~map_res")){
    	ros::param::get("~map_res",map_res);
  	}	
	if (ros::param::has("~map_length")){
    	ros::param::get("~map_length",map_length);
  	}	
	if (ros::param::has("~map_width")){
    	ros::param::get("~map_width",map_width);
	}
	if (ros::param::has("~use_planes")){
    	ros::param::get("~use_planes",use_planes);
	}
	if (ros::param::has("~show_timing")){
    	ros::param::get("~show_timing",show_timing);
	}
	if (ros::param::has("~fixed_map")){
    	ros::param::get("~fixed_map",fixed_map);
	}
	float vehicle_mass = 34251.0f;
	float vehicle_bumper_height = 0.41f;
	float vehicle_tire_radius = 0.4f;
	float vehicle_vci1 = 25.0f;
	float vehicle_max_slope = 0.55f;
	float vehicle_roof_height = 3.5f;
	if (ros::param::has("~vehicle_mass")){
    	ros::param::get("~vehicle_mass",vehicle_mass);
	}
	if (ros::param::has("~vehicle_bumper_height")){
    	ros::param::get("~vehicle_bumper_height",vehicle_bumper_height);
	}
	if (ros::param::has("~vehicle_tire_radius")){
    	ros::param::get("~vehicle_tire_radius",vehicle_tire_radius);
	}
	if (ros::param::has("~vehicle_vci1")){
    	ros::param::get("~vehicle_vci1",vehicle_vci1);
	}
	if (ros::param::has("~vehicle_max_slope")){
    	ros::param::get("~vehicle_max_slope",vehicle_max_slope);
	}
	if (ros::param::has("~vehicle_roof_height")){
    	ros::param::get("~vehicle_roof_height",vehicle_roof_height);
	}
	float default_traversability = 0.6f;
	if (ros::param::has("~default_traversability")){
    	ros::param::get("~default_traversability",default_traversability);
	}

	float slope_coeff = 0.25f;
	if (ros::param::has("~slope_coeff")){
    	ros::param::get("~slope_coeff",slope_coeff);
	}
	float slope_exponent = 2.0f;
	if (ros::param::has("~slope_exponent")){
    	ros::param::get("~slope_exponent",slope_exponent);
	}
	float soil_coeff = 0.0f;
	if (ros::param::has("~soil_coeff")){
    	ros::param::get("~soil_coeff",soil_coeff);
	}
	float soil_exponent = 1.0f;
	if (ros::param::has("~soil_exponent")){
    	ros::param::get("~soil_exponent",soil_exponent);
	}
	float veg_coeff = 1.0f;
	if (ros::param::has("~veg_coeff")){
    	ros::param::get("~veg_coeff",veg_coeff);
	}
	float veg_exponent = 1.0f;
	if (ros::param::has("~veg_exponent")){
    	ros::param::get("~veg_exponent",veg_exponent);
	}
	float roughness_coeff = 1.0f;
	if (ros::param::has("~roughness_coeff")){
    	ros::param::get("~roughness_coeff",roughness_coeff);
	}
	float roughness_exponent = 1.0f;
	if (ros::param::has("~roughness_exponent")){
    	ros::param::get("~roughness_exponent",roughness_exponent);
	}
	bool show_traversability = false;
	if (ros::param::has("~show_traversability")){
    	ros::param::get("~show_traversability",show_traversability);
	}
	bool show_confidence = false;
	if (ros::param::has("~show_confidence")){
    	ros::param::get("~show_confidence",show_confidence);
	}
	bool show_ground = false;
	if (ros::param::has("~show_ground")){
    	ros::param::get("~show_ground",show_ground);
	}
	bool show_roughness = false;
	if (ros::param::has("~show_roughness")){
    	ros::param::get("~show_roughness",show_roughness);
	}
	bool show_veg = false;
	if (ros::param::has("~show_veg")){
    	ros::param::get("~show_veg",show_veg);
	}
	bool show_slope = false;
	if (ros::param::has("~show_slope")){
    	ros::param::get("~show_slope",show_slope);
	}

	bool save_plots = false;
	if (ros::param::has("~save_plots")){
    	ros::param::get("~save_plots",save_plots);
	}

	double max_time = std::numeric_limits<double>::max();
	if (ros::param::has("~max_time")){
    	ros::param::get("~max_time",max_time);
	}


	// Create a vehicle for calculating traversability
	traverselib::Vehicle vehicle;
	vehicle.SetParams(vehicle_mass, vehicle_bumper_height, vehicle_tire_radius, vehicle_vci1,vehicle_max_slope, vehicle_roof_height);
	vehicle.SetSlopeCoeff(slope_coeff);
	vehicle.SetSlopeExponent(slope_exponent);
	vehicle.SetSoilCoeff(soil_coeff);
	vehicle.SetSoilExponent(soil_exponent);
	vehicle.SetVegCoeff(veg_coeff);
	vehicle.SetVegExponent(veg_exponent);
	vehicle.SetRoughnessCoeff(roughness_coeff);
	vehicle.SetRoughnessExponent(roughness_exponent);

	// Create a traversability model
	traverselib::VoxelGrid grid;
	grid.SetDefaultTraversability(default_traversability);
	grid.SetAveragingRadius(3.0f*map_res);
	grid.UsePlaneFitting(use_planes);
	grid.SetPrintTimingInfo(show_timing);

	int frame_count = 0;
	double t0 = 0.0f;
	double elapsed_time = 0.0;
	std::ofstream fout("pose_log.txt");
	// Start the ROS loop
	while (ros::ok() && elapsed_time<max_time){
		if (odom_rcvd && points_rcvd){
			if (frame_count==0){
				t0 = ros::Time::now().toSec();
			}
			// Initialized the traversability model the first
			// iteration based on the vehicle position
			if (!grid.Initialized()){
				glm::vec3 llc(current_position.x-0.5f*map_width, current_position.y-0.5*map_length, current_position.z-5.0);
				glm::vec3 urc(current_position.x+0.5f*map_width, current_position.y+0.5f*map_length, current_position.z+10.0);
				grid.Initialize(llc,urc,map_res);
				grid.SetVehicle(vehicle);
			}
			else if (!fixed_map){
				glm::vec3 llc(current_position.x-0.5f*map_width, current_position.y-0.5*map_length, current_position.z-5.0);
				glm::vec3 urc(current_position.x+0.5f*map_width, current_position.y+0.5f*map_length, current_position.z+10.0);
				grid.Move(llc, urc);
			}
			fout<<elapsed_time<<" "<<current_position.x<<" "<<current_position.y<<" "<<current_position.z<<std::endl;

			grid.AddRegisteredPoints(current_points, current_position);

			// Plot results
			std::string file_prefix = ToString(frame_count, 5);
			
			if (show_confidence){
				grid.PlotConfidence();
				//if (save_plots) grid.SaveConfidencePlot(file_prefix+"_conf.bmp");
			}
			if (show_ground) {
				grid.PlotGround();
				//if (save_plots) grid.SaveGroundPlot(file_prefix+"_ground.bmp");
			}
			if (show_roughness){
				grid.PlotRoughness();
				//if (save_plots) grid.SaveRoughPlot(file_prefix+"_rough.bmp");
			}
			if (show_veg) {
				grid.PlotVegDensity();
				//if (save_plots) grid.SaveVegDensityPlot(file_prefix+"_veg.bmp");
			}
			if (show_slope) {
				grid.PlotSlope();
				//if (save_plots) grid.SaveSlopePlot(file_prefix+"_slope.bmp");
			}
			if (show_traversability) {
				grid.PlotTraversability();
				//if (save_plots) grid.SaveTraversabilityPlot(file_prefix+"_trav.bmp");
			}
			if (save_plots){
				grid.SaveConfidencePlot(file_prefix+"_conf.bmp");
				grid.SaveGroundPlot(file_prefix+"_ground.bmp");
				grid.SaveRoughPlot(file_prefix+"_rough.bmp");
				grid.SaveVegDensityPlot(file_prefix+"_veg.bmp");
				grid.SaveSlopePlot(file_prefix+"_slope.bmp");
				grid.SaveTraversabilityPlot(file_prefix+"_trav.bmp");
			}
			nav_msgs::OccupancyGrid occ_grid = grid.GetTraversabilityAsOccupancyGrid(false);
			occ_grid.header = current_pose.header;
			grid_pub.publish(occ_grid);

			nav_msgs::OccupancyGrid occ_grid_vis = grid.GetTraversabilityAsOccupancyGrid(true);
			occ_grid_vis.header = current_pose.header;
			grid_pub_vis.publish(occ_grid_vis);

			points_rcvd = false;
			frame_count++;
			elapsed_time = ros::Time::now().toSec() - t0;
		}
		ros::spinOnce();
	}
	fout.close();
	return 0;
}
