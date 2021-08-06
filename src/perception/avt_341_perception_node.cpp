// c++ includes
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
// ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "sensor_msgs/point_cloud_conversion.h"
// avt_341 includes
#include "avt_341/perception/elevation_grid.h"

avt_341::perception::ElevationGrid grid;
avt_341::msg::Odometry current_pose;
bool grid_created = false;
bool odom_rcvd = false;
std::vector<avt_341::msg::Odometry> current_pose_list;
bool use_registered = true;
float overhead_clearance = 100.0f;

void PointCloudCallbackRegistered(avt_341::msg::PointCloud2Ptr rcv_cloud){
	// assumes point cloud is already registered to odom frame
	avt_341::msg::PointCloud point_cloud;
	
  bool converted = sensor_msgs::convertPointCloud2ToPointCloud(*rcv_cloud,point_cloud);
	if (converted && odom_rcvd){
		std::vector<avt_341::msg::Point32> points;
		for (int p=0;p<point_cloud.points.size();p++){
			avt_341::msg::Point32 tp;
			tp.x = point_cloud.points[p].x;
			tp.y = point_cloud.points[p].y;
			tp.z = point_cloud.points[p].z;
			if ( !(tp.x==0.0 && tp.y==0.0) && !std::isnan(tp.x) && (tp.z-current_pose.pose.pose.position.z)<overhead_clearance ){
				points.push_back(tp);
				
			}
		}
		point_cloud.points.clear();
		point_cloud.points = points;
		grid.AddPoints(point_cloud);
		grid_created = true;
	}
}

void PointCloudCallbackUnregistered(avt_341::msg::PointCloud2Ptr rcv_cloud){
	avt_341::msg::PointCloud point_cloud;
	
  bool converted = sensor_msgs::convertPointCloud2ToPointCloud(*rcv_cloud,point_cloud);
	double dt = 1.0;
	avt_341::msg::Odometry pose_to_use;
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
		std::vector<avt_341::msg::Point32> points;
		for (int p=0;p<point_cloud.points.size();p++){
			tf::Vector3 v;
			v = tf::Vector3(point_cloud.points[p].x, point_cloud.points[p].y,point_cloud.points[p].z);
			double r = sqrt( pow(v.x()-origin.x(), 2)+pow(v.y()-origin.y(), 2)+pow(v.z()-origin.z(), 2));
			if (v.x()!=0.0 && v.y()!=0.0 &&!std::isnan(v.x())){
				tf::Vector3 vp = (R*v) + origin;
				avt_341::msg::Point32 tp;
				tp.x = vp.x();
				tp.y = vp.y();
				tp.z = vp.z();
				if ( (tp.z-current_pose.pose.pose.position.z)<overhead_clearance){
					points.push_back(tp);
				}
				
			}
		}
		point_cloud.points.clear();
		point_cloud.points = points;
		grid.AddPoints(point_cloud);
		grid_created = true;
	}
}
void PointCloudCallback(avt_341::msg::PointCloud2Ptr rcv_cloud){
	if (use_registered){
		PointCloudCallbackRegistered(rcv_cloud);
	}
	else{
		PointCloudCallbackUnregistered(rcv_cloud);
	}
}

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom){
	current_pose = *rcv_odom;
	//std::cout<<"Vehicle positoin = "<<current_pose.pose.pose.position.x<<" "<<current_pose.pose.pose.position.y<<" "<<current_pose.pose.pose.position.z<<std::endl;
	odom_rcvd = true;
	current_pose_list.push_back(current_pose);
	if (current_pose_list.size()>50) current_pose_list.erase(current_pose_list.begin());
}

int main(int argc, char *argv[]) {

	grid_created = false;

	auto n = avt_341::node::init_node(argc, argv, "avt_341_perception_node");
	auto pc_sub = n->create_subscription<avt_341::msg::PointCloud2>("avt_341/points",2,PointCloudCallback);
    auto odom_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry",10, OdometryCallback);
    auto grid_pub = n->create_publisher<avt_341::msg::OccupancyGrid>("avt_341/occupancy_grid", 1);

    float grid_width, grid_height;
    n->get_parameter("~grid_width", grid_width, 200.0f);
    n->get_parameter("~grid_height", grid_height, 200.0f);
    grid.SetSize(grid_width,grid_height);

    float grid_res, grid_llx, grid_lly, warmup_time, thresh;
    bool use_elevation;

    n->get_parameter("~grid_res", grid_res, 1.0f);
    n->get_parameter("~grid_llx", grid_llx, -100.0f);
    n->get_parameter("~grid_lly", grid_lly, -100.0f);

    n->get_parameter("~warmup_time", warmup_time, 1.0f);
    n->get_parameter("~slope_threshold", thresh, 1.0f);
    n->get_parameter("~use_elevation", use_elevation, false);
    n->get_parameter("~use_registered", use_registered, true);
    n->get_parameter("~overhead_clearance", overhead_clearance, 100.0f);

	grid.SetSlopeThreshold(thresh);
	grid.SetRes(grid_res);
	grid.SetCorner(grid_llx,grid_lly);
	
	double start_time = n->get_now_seconds();
	avt_341::node::Rate rate(100.0);
	while (avt_341::node::ok()){
		double elapsed_time = (n->get_now_seconds()-start_time);
		if (grid_created && elapsed_time > warmup_time) {
			avt_341::msg::OccupancyGrid grd;
			if (use_elevation){
				grd = grid.GetGrid("elevation");
			}  
			else {
				grd = grid.GetGrid("slope");
			}
			grid_pub->publish(grd);
		}
		n->spin_some();
		rate.sleep();
	}

	return 0;
}