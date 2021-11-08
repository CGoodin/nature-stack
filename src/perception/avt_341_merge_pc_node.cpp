//ros includes
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
// tf includes
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// pcl includes
#include <pcl_ros/point_cloud.h>
// avt includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

sensor_msgs::PointCloud2 cloud_in, front_cloud_in;
bool lidar_rcvd = false;
bool front_lidar_rcvd = false;

void PointCloudCallback(avt_341::msg::PointCloud2Ptr rcv_cloud){
	cloud_in = *rcv_cloud;
	lidar_rcvd = true;
}

void PointCloudFrontCallback(const sensor_msgs::PointCloud2::ConstPtr& rcv_cloud){
	front_cloud_in = *rcv_cloud;
	front_lidar_rcvd = true;
}

int main(int argc, char** argv) {
	//ros::init(argc, argv, "avt_341_merge_pc_node");

	auto n = avt_341::node::init_node(argc, argv, "avt_341_merge_pc_node");
	auto pc_sub = n->create_subscription<avt_341::msg::PointCloud2>("avt_341/top_lidar",2,PointCloudCallback);
	auto pc_sub_front = n->create_subscription<avt_341::msg::PointCloud2>("avt_341/front_lidar",2,PointCloudFrontCallback);

 	auto merged_lidar_pub = n->create_publisher<avt_341::msg::PointCloud2>("avt_341/points", 1);

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	sensor_msgs::PointCloud2 cloud_out, front_cloud_out;

	avt_341::node::Rate rate(10.0);
	while (avt_341::node::ok()){
		geometry_msgs::TransformStamped transformStamped, front_transformStamped;
		try {
			sensor_msgs::PointCloud2 merged_cloud;
			bool publish_merged = false;
			if (lidar_rcvd){
				//tfListener.waitForTransform("odom", "lidar", ros::Time::now(), ros::Duration(3.0));
				std::string error_msg;
				if  (tfBuffer.canTransform("odom", "lidar",cloud_in.header.stamp, ros::Duration(15.0), &error_msg)){
	
					//std::cout<<"TRANSORM RECIEVED "<<error_msg << std::endl;
					//transformStamped = tfBuffer.lookupTransform("odom", "lidar", cloud_in.header.stamp);
					transformStamped = tfBuffer.lookupTransform("map", "lidar", cloud_in.header.stamp);
					//transformStamped = tfBuffer.lookupTransform("lidar", "odom", ros::Time(0));
					//transformStamped = tfBuffer.lookupTransform("lidar", "odom", cloud_in.header.stamp);
					tf2::doTransform(cloud_in, cloud_out, transformStamped);
					pcl::concatenatePointCloud(cloud_out, merged_cloud, merged_cloud);
					publish_merged = true;
				}
				else{
					//std::cout<<"NO TRANSFORM RECIEVED "<<error_msg<<std::endl;
				}
			}
			/*if (front_lidar_rcvd){
				front_transformStamped = tfBuffer.lookupTransform("odom", "front_lidar", ros::Time(0));
				tf2::doTransform(front_cloud_in, front_cloud_out, front_transformStamped);
				pcl::concatenatePointCloud(front_cloud_out, merged_cloud, merged_cloud);
			}*/
			if (publish_merged){
			//if (lidar_rcvd || front_lidar_rcvd) {
				//std::cout<<"Publishing merged point cloud "<<std::endl;
				merged_cloud.header.stamp = ros::Time::now();
				merged_lidar_pub->publish(merged_cloud);
			}
		} 
		catch (tf2::TransformException &ex) {
				continue;
		}
		n->spin_some();
		rate.sleep();
	}
	return 0;
}