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
bool grid_created = false;
bool odom_rcvd = false;
std::vector<nav_msgs::Odometry> current_pose_list;
bool use_registered = true;

void PointCloudCallbackRegistered(const sensor_msgs::PointCloud2::ConstPtr& rcv_cloud){
	// assumes point cloud is already registered to odom frame
	sensor_msgs::PointCloud point_cloud;
	
  bool converted = sensor_msgs::convertPointCloud2ToPointCloud(*rcv_cloud,point_cloud);
	if (converted && odom_rcvd){
		std::vector<geometry_msgs::Point32> points;
		for (int p=0;p<point_cloud.points.size();p++){
			geometry_msgs::Point32 tp;
			tp.x = point_cloud.points[p].x;
			tp.y = point_cloud.points[p].y;
			tp.z = point_cloud.points[p].z;
			if ( !(tp.x==0.0 && tp.y==0.0) && !std::isnan(tp.x)){
				points.push_back(tp);
				
			}
		}
		point_cloud.points.clear();
		point_cloud.points = points;
		grid.AddPoints(point_cloud);
		grid_created = true;
	}
}

void PointCloudCallbackUnregistered(const sensor_msgs::PointCloud2::ConstPtr& rcv_cloud){
	sensor_msgs::PointCloud point_cloud;
	
  bool converted = sensor_msgs::convertPointCloud2ToPointCloud(*rcv_cloud,point_cloud);
	double dt = 1.0;
	nav_msgs::Odometry pose_to_use;
	for (int i=0;i<current_pose_list.size();i++){
		double dt_this = fabs(current_pose_list[i].header.stamp.toSec() - rcv_cloud->header.stamp.toSec());
		if (dt_this<dt){
			pose_to_use = current_pose_list[i];
			dt = dt_this;
		}
	}
	if (converted && fabs(dt)<0.01 && odom_rcvd){
		tf::Quaternion q(pose_to_use.pose.pose.orientation.x, pose_to_use.pose.pose.orientation.y, pose_to_use.pose.pose.orientation.z, current_pose.pose.pose.orientation.w);
		tf::Matrix3x3 R(q);
		tf::Vector3 origin(pose_to_use.pose.pose.position.x, pose_to_use.pose.pose.position.y, pose_to_use.pose.pose.position.z);
		std::vector<geometry_msgs::Point32> points;
		for (int p=0;p<point_cloud.points.size();p++){
			tf::Vector3 v;
			v = tf::Vector3(point_cloud.points[p].x, point_cloud.points[p].y,point_cloud.points[p].z);
			double r = sqrt( pow(v.x()-origin.x(), 2)+pow(v.y()-origin.y(), 2)+pow(v.z()-origin.z(), 2));
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
void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& rcv_cloud){
	if (use_registered){
		PointCloudCallbackRegistered(rcv_cloud);
	}
	else{
		PointCloudCallbackUnregistered(rcv_cloud);
	}
}

void OdometryCallback(const nav_msgs::Odometry::ConstPtr& rcv_odom){
	current_pose = *rcv_odom;
	odom_rcvd = true;
	current_pose_list.push_back(current_pose);
	if (current_pose_list.size()>50) current_pose_list.erase(current_pose_list.begin());
}

int main(int argc, char *argv[]) {

	grid_created = false;

	ros::init(argc, argv, "avt_341_perception_node");
	
	ros::NodeHandle n;
	ros::Subscriber pc_sub = n.subscribe("avt_341/points",2,PointCloudCallback);
	ros::Subscriber odom_sub = n.subscribe("avt_341/odometry",10, OdometryCallback);
	ros::Publisher grid_pub = n.advertise<nav_msgs::OccupancyGrid>("avt_341/occupancy_grid", 1);

	if (ros::param::has("~grid_width") && ros::param::has("~grid_height")){
		float grid_width, grid_height;
		ros::param::get("~grid_width",grid_width);
		ros::param::get("~grid_height",grid_height);
		grid.SetSize(grid_width,grid_height);
	}
	else{
		float grid_size = 500.0f;
		if (ros::param::has("~grid_size")){
    		ros::param::get("~grid_size",grid_size);
  		}	
		grid.SetSize(grid_size);
	}


	float grid_res = 1.0f;
	if (ros::param::has("~grid_res")){
    	ros::param::get("~grid_res",grid_res);
  	}	
  	float grid_llx=-250.0f; 
  	if (ros::param::has("~grid_llx")){
    	ros::param::get("~grid_llx",grid_llx);
  	}	
  	float grid_lly=-250.0f; 
  	if (ros::param::has("~grid_lly")){
    	ros::param::get("~grid_lly",grid_lly);
 	 }	
	float warmup_time = 1.0f;
	if (ros::param::has("~warmup_time")){
    	ros::param::get("~warmup_time",warmup_time);
  	}	
	float thresh = 1.0f;
	if (ros::param::has("~threshold")){
		ros::param::get("~threshold",thresh);
	}
	bool use_elevation = false;
	if (ros::param::has("~use_elevation")){
		ros::param::get("~use_elevation",use_elevation);
	}
	if (ros::param::has("~use_registered")){
		ros::param::get("~use_registered",use_registered);
	}

	grid.SetSlopeThreshold(thresh);
	grid.SetRes(grid_res);
	grid.SetCorner(grid_llx,grid_lly);
	
	double start_time = ros::Time::now().toSec();
	ros::Rate rate(100.0);
	while (ros::ok()){
		double elapsed_time = (ros::Time::now().toSec()-start_time); 
		if (grid_created && elapsed_time > warmup_time) {
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