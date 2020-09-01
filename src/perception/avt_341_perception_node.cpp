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
#include "tf/LinearMath/Transform.h"
// avt_341 includes
#include "avt_341/perception/elevation_grid.h"

avt_341::perception::ElevationGrid grid;
nav_msgs::Odometry current_pose;
bool grid_created;

void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& rcv_cloud){
	sensor_msgs::PointCloud point_cloud;
    bool converted = sensor_msgs::convertPointCloud2ToPointCloud(*rcv_cloud,point_cloud);
	if (converted){
		tf::Quaternion q(current_pose.pose.pose.orientation.x, current_pose.pose.pose.orientation.y, current_pose.pose.pose.orientation.z, current_pose.pose.pose.orientation.w);
		tf::Matrix3x3 R(q);
		tf::Vector3 origin(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, current_pose.pose.pose.position.z);
		std::vector<geometry_msgs::Point32> points;
		for (int p=0;p<point_cloud.points.size();p++){
			tf::Vector3 v;
			v = tf::Vector3(point_cloud.points[p].x, point_cloud.points[p].y,point_cloud.points[p].z);
			if (v.x()!=0.0 && v.y()!=0.0 &&!std::isnan(v.x())){
				tf::Vector3 vp = (R*v) + origin;
				geometry_msgs::Point32 tp;
				tp.x = vp.x();
				tp.y = vp.y();
				tp.z = vp.z();
				points.push_back(tp);
				
			}
		}
		point_cloud.points.clear();
		point_cloud.points = points;
		grid.AddPoints(point_cloud);
		grid_created = true;
	}
}

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& rcv_odom){
	current_pose = *rcv_odom;
}

int main(int argc, char *argv[]) {

	grid_created = false;

	ros::init(argc, argv, "avt_341_perception_node");
	
	ros::NodeHandle n;
	ros::Subscriber pc_sub = n.subscribe("avt_341/points",2,PointCloudCallback);
	ros::Subscriber odom_sub = n.subscribe("avt_341/odometry",10, OdometryCallback);
	ros::Publisher grid_pub = n.advertise<nav_msgs::OccupancyGrid>("avt_341/occupancy_grid", 1);

	float grid_size = 500.0f;
	if (ros::param::has("~grid_size")){
    	ros::param::get("~grid_size",grid_size);
  	}	
	float grid_res = 1.0f;
	if (ros::param::has("~grid_res")){
    	ros::param::get("~grid_res",grid_res);
  	}	
	//bool display = false;
	//if (ros::param::has("~display")){
//		ros::param::get("~display",display);
	//}
	float thresh = 1.0f;
	if (ros::param::has("~threshold")){
		ros::param::get("~threshold",thresh);
	}
	bool use_elevation = false;
	if (ros::param::has("~use_elevation")){
		ros::param::get("~use_elevation",use_elevation);
	}

	grid.SetSlopeThreshold(thresh);
	grid.SetSize(grid_size);
	grid.SetRes(grid_res);
	ros::Rate rate(100.0);
	while (ros::ok()){
		if (grid_created) {
			//if (display)grid.Display();
			nav_msgs::OccupancyGrid grd;
			if (use_elevation){
				grd = grid.GetGrid("elevation");
			}  
			else {
				grd = grid.GetGrid("slope");
			}
			grid_pub.publish(grd);
		}
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}