// c++ includes
#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
// ros includes
#include "nature/node/ros_types.h"
#include "nature/node/node_proxy.h"
// nature includes
#include "nature/perception/elevation_grid.h"

nature::perception::ElevationGrid grid;
nature::msg::Odometry current_pose;
bool grid_created = false;
bool odom_rcvd = false;
std::vector<nature::msg::Odometry> current_pose_list;
bool use_registered = true;
float overhead_clearance = 100.0f;
double time_register_window = 0.02;
bool cull_lidar_points = false;
float cull_lidar_points_dist_sqr = 10000.0f;
float blanking_distance = 0.0f;
float blanking_distance_sqr = 0.0f;

double CalcLidarPointToRobotDistanceSquared(const nature::msg::Point& odom_pose, const nature::msg::Point32& point){
	double dx = odom_pose.x - point.x;
	double dy = odom_pose.y - point.y;
	double dz = odom_pose.z - point.z;
	return (dx*dx + dy*dy + dz*dz);
}

double GetPoseToUse(nature::msg::Odometry & pose_to_use, nature::msg::PointCloud2Ptr rcv_cloud){
  double dt = 1.0;
  for (int i=0;i<current_pose_list.size();i++){
    double dt_this = fabs(nature::node::seconds_from_header(current_pose_list[i].header) - nature::node::seconds_from_header(rcv_cloud->header));
    if (dt_this<dt){
      pose_to_use = current_pose_list[i];
      dt = dt_this;
    }
  }
	return dt;
}

void PointCloudCallbackRegistered(nature::msg::PointCloud2Ptr rcv_cloud){
	// assumes point cloud is already registered to odom frame
	nature::msg::PointCloud point_cloud;
	
  bool converted = sensor_msgs::convertPointCloud2ToPointCloud(*rcv_cloud,point_cloud);
	if (converted && odom_rcvd){
		std::vector<nature::msg::Point32> points;
		std::vector<std::vector<float>> channel_values;
		for(int c = 0; c < point_cloud.channels.size(); c++){
			channel_values.push_back(std::vector<float>());
		}
		for (int p=0;p<point_cloud.points.size();p++){
			nature::msg::Point32 tp;
			tp.x = point_cloud.points[p].x;
			tp.y = point_cloud.points[p].y;
			tp.z = point_cloud.points[p].z;
			if ( !(tp.x==0.0 && tp.y==0.0) && !std::isnan(tp.x) && (tp.z-current_pose.pose.pose.position.z)<overhead_clearance ){

				bool add_point = true;
				nature::msg::Odometry pose_to_use;
				GetPoseToUse(pose_to_use, rcv_cloud);
				double dr2 = CalcLidarPointToRobotDistanceSquared(pose_to_use.pose.pose.position, tp); 
				if (cull_lidar_points){
					add_point = dr2 < cull_lidar_points_dist_sqr;
				}
				if(add_point && dr2> blanking_distance_sqr){
					points.push_back(tp);
					for(int c = 0; c < point_cloud.channels.size(); c++){
						channel_values[c].push_back(point_cloud.channels[c].values[p]);
					}
				}

			}
		}
		point_cloud.points.clear();
		point_cloud.points = points;
		for(int c = 0; c < point_cloud.channels.size(); c++){
			point_cloud.channels[c].values = channel_values[c];
		}
		grid.AddPoints(point_cloud);
		grid_created = true;
	}
}

void PointCloudCallbackUnregistered(nature::msg::PointCloud2Ptr rcv_cloud){
	nature::msg::PointCloud point_cloud;
	
	bool converted = sensor_msgs::convertPointCloud2ToPointCloud(*rcv_cloud,point_cloud);

	nature::msg::Odometry pose_to_use;
	double dt = GetPoseToUse(pose_to_use, rcv_cloud);
	std::vector<std::vector<float>> channel_values;
	for(int c = 0; c < point_cloud.channels.size(); c++){
		channel_values.push_back(std::vector<float>());
	}
	if (converted && fabs(dt)<time_register_window && odom_rcvd){
    nature::msg_tf::Quaternion q(pose_to_use.pose.pose.orientation.x, pose_to_use.pose.pose.orientation.y, pose_to_use.pose.pose.orientation.z, current_pose.pose.pose.orientation.w);
    nature::msg_tf::Matrix3x3 R(q);
    nature::msg_tf::Vector3 origin(pose_to_use.pose.pose.position.x, pose_to_use.pose.pose.position.y, pose_to_use.pose.pose.position.z);
		std::vector<nature::msg::Point32> points;
		for (int p=0;p<point_cloud.points.size();p++){
      nature::msg_tf::Vector3 v;
			v = nature::msg_tf::Vector3(point_cloud.points[p].x, point_cloud.points[p].y,point_cloud.points[p].z);
			//double r = sqrt( pow(v.x()-origin.x(), 2)+pow(v.y()-origin.y(), 2)+pow(v.z()-origin.z(), 2));
			if (v.x()!=0.0 && v.y()!=0.0 &&!std::isnan(v.x())){
        nature::msg_tf::Vector3 vp = (R*v) + origin;
				nature::msg::Point32 tp;
				tp.x = vp.x();
				tp.y = vp.y();
				tp.z = vp.z();
				float dr2 = CalcLidarPointToRobotDistanceSquared(pose_to_use.pose.pose.position, tp);
				if ( (tp.z-current_pose.pose.pose.position.z)<overhead_clearance &&
             		(!cull_lidar_points || dr2 < cull_lidar_points_dist_sqr)
								 && dr2 > blanking_distance_sqr){
					points.push_back(tp);
					for(int c = 0; c < point_cloud.channels.size(); c++){
						channel_values[c].push_back(point_cloud.channels[c].values[p]);
					}
				}
				
			}
		}
		point_cloud.points.clear();
		point_cloud.points = points;
		for(int c = 0; c < point_cloud.channels.size(); c++){
			point_cloud.channels[c].values = channel_values[c];
		}
		grid.AddPoints(point_cloud);
		grid_created = true;
	}
}
void PointCloudCallback(nature::msg::PointCloud2Ptr rcv_cloud){
	if (use_registered){
		PointCloudCallbackRegistered(rcv_cloud);
	}
	else{
		PointCloudCallbackUnregistered(rcv_cloud);
	}
}

void OdometryCallback(nature::msg::OdometryPtr rcv_odom){
	current_pose = *rcv_odom;
	odom_rcvd = true;
	current_pose_list.push_back(current_pose);
	if (current_pose_list.size()>50) current_pose_list.erase(current_pose_list.begin());
}

int main(int argc, char *argv[]) {

	grid_created = false;

	auto n = nature::node::init_node(argc, argv, "nature_perception_node");
	auto pc_sub = n->create_subscription<nature::msg::PointCloud2>("nature/points",2,PointCloudCallback);
    auto odom_sub = n->create_subscription<nature::msg::Odometry>("nature/odometry",10, OdometryCallback);
    auto grid_pub = n->create_publisher<nature::msg::OccupancyGrid>("nature/occupancy_grid", 1);
    auto grid_segmentation_pub = n->create_publisher<nature::msg::OccupancyGrid>("nature/segmentation_grid", 1);

    float grid_width, grid_height;
    n->get_parameter("~grid_width", grid_width, 200.0f);
    n->get_parameter("~grid_height", grid_height, 200.0f);
    grid.SetSize(grid_width,grid_height);

    float grid_res, grid_llx, grid_lly, warmup_time, thresh, grid_dilate_x, grid_dilate_y, grid_dilate_proportion;
    bool use_elevation, grid_dilate, persistent_obstacles;
    std::string display;


	n->get_parameter("~blanking_distance", blanking_distance, 0.0f);
	blanking_distance_sqr = blanking_distance*blanking_distance;
	n->get_parameter("~grid_res", grid_res, 1.0f);
	n->get_parameter("~grid_llx", grid_llx, -100.0f);
	n->get_parameter("~grid_lly", grid_lly, -100.0f);
	n->get_parameter("~time_register_window", time_register_window, 0.02);
	n->get_parameter("~warmup_time", warmup_time, 1.0f);
	n->get_parameter("~slope_threshold", thresh, 1.0f);
	n->get_parameter("~use_elevation", use_elevation, false);
	n->get_parameter("~persistent_obstacles", persistent_obstacles, false);
	n->get_parameter("~use_registered", use_registered, true);
	n->get_parameter("~grid_dilate", grid_dilate, true);
	n->get_parameter("~grid_dilate_x", grid_dilate_x, 2.0f);
	n->get_parameter("~grid_dilate_y", grid_dilate_y, 2.0f);
	n->get_parameter("~grid_dilate_proportion", grid_dilate_proportion, 0.8f);
	n->get_parameter("~overhead_clearance", overhead_clearance, 100.0f);
	n->get_parameter("~display", display, std::string("image"));
	bool stitch_points;
	n->get_parameter("~stitch_lidar_points", stitch_points, true);
	bool filter_highest_lidar;
	n->get_parameter("~filter_highest_lidar", filter_highest_lidar, false);
    float cull_lidar_points_dist;
    n->get_parameter("~cull_lidar", cull_lidar_points, false);
    n->get_parameter("~cull_lidar_dist", cull_lidar_points_dist, 100.0f);
    cull_lidar_points_dist_sqr = cull_lidar_points_dist * cull_lidar_points_dist;


  bool use_rviz = display == "rviz";
    std::shared_ptr<nature::node::Publisher<nature::msg::OccupancyGrid>> grid_pub_vis;
    std::shared_ptr<nature::node::Publisher<nature::msg::OccupancyGrid>> grid_segmentation_vis_pub;
    if(use_rviz){
      grid_pub_vis = n->create_publisher<nature::msg::OccupancyGrid>("nature/occupancy_grid_vis", 1);
      grid_segmentation_vis_pub = n->create_publisher<nature::msg::OccupancyGrid>("nature/segmentation_grid_vis", 1);
    }

	grid.SetSlopeThreshold(thresh);
	grid.SetRes(grid_res);
	grid.SetCorner(grid_llx,grid_lly);
	grid.SetUseElevation(use_elevation);
	grid.SetDilation(grid_dilate, grid_dilate_x, grid_dilate_y, grid_dilate_proportion);
	grid.SetStitchPoints(stitch_points);
	grid.SetFilterHighest(filter_highest_lidar);
	grid.SetPersistentObstacles(persistent_obstacles);

	double start_time = n->get_now_seconds();
	nature::node::Rate rate(100.0);
  int nloops = 0;
	while (nature::node::ok()){
		double elapsed_time = (n->get_now_seconds()-start_time);
		if (grid_created && elapsed_time > warmup_time) {
			nature::msg::OccupancyGrid grd;
      		grd = grid.GetGrid();
			grd.header.stamp = n->get_stamp();
			grid_pub->publish(grd);

			if(grid.has_segmentation()){
				grd = grid.GetGrid(false, true);
				grd.header.stamp = n->get_stamp();
				grid_segmentation_pub->publish(grd);
			}

			if(use_rviz && nloops % 10 == 0){
				nature::msg::OccupancyGrid grd_vis = grid.GetGrid( true);
				grd_vis.header.stamp = n->get_stamp();
				grid_pub_vis->publish(grd_vis);
				if(grid.has_segmentation()){
					grd_vis = grid.GetGrid(true, true);
					grd_vis.header.stamp = n->get_stamp();
					grid_segmentation_vis_pub->publish(grd_vis);
				}
			}
			nloops++;

		}
		n->spin_some();
		rate.sleep();
	}

	return 0;
}