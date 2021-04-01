//ros includes
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/PointCloud2.h"
#include "rosgraph_msgs/Clock.h"
// pcl includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

geometry_msgs::Twist twist;
void TwistCallback(const geometry_msgs::Twist::ConstPtr& rcv_msg){
	twist.linear.x = rcv_msg->linear.x; // throttle
	twist.linear.y = rcv_msg->linear.y; // braking
	twist.angular.z = rcv_msg->angular.z; // steering
}

int main(int argc, char **argv){

	ros::init(argc,argv,"avt_341_simulation_test_node");
	ros::NodeHandle n;

	ros::Subscriber twist_sub = n.subscribe("avt_341/cmd_vel",1,TwistCallback);

	ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("avt_341/points",1);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("avt_341/odometry",1);


	// determine if sim time will be used
	bool use_sim_time = false;
	ros::Publisher clock_pub;
	if (n.hasParam("use_sim_time")){
		n.getParam("use_sim_time",use_sim_time);
	}
	if (use_sim_time){
		clock_pub = n.advertise<rosgraph_msgs::Clock>("clock", 1);
	}

	// create and populate the odometry message that will be published
	nav_msgs::Odometry odom_msg;
	odom_msg.header.frame_id = "odom";
	odom_msg.header.seq = 0;
	odom_msg.pose.pose.position.x = 0.0;
	odom_msg.pose.pose.position.y = 0.0;
	odom_msg.pose.pose.position.z = 1.0;
	odom_msg.pose.pose.orientation.w = 1.0;
	odom_msg.pose.pose.orientation.x = 0.0;
	odom_msg.pose.pose.orientation.y = 0.0;
	odom_msg.pose.pose.orientation.z = 0.0;
	odom_msg.twist.twist.linear.x = 0.0;
	odom_msg.twist.twist.linear.y = 0.0;
	odom_msg.twist.twist.linear.z = 0.0;
	odom_msg.twist.twist.angular.x = 0.0;
	odom_msg.twist.twist.angular.y = 0.0;
	odom_msg.twist.twist.angular.z = 0.0;

	// create and populate the point cloud message that will be published
	sensor_msgs::PointCloud2 pc2; 
	pc2.header.frame_id = "odom";
	pc2.header.seq = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> point_cloud;
	point_cloud.push_back(pcl::PointXYZ(50.0, 0.0, 0.0));
	pcl::toROSMsg(point_cloud, pc2);

	//odometry published at 100 Hz, point clout at 10 Hz
	double dt = 0.01;
	ros::Rate rate(1.0/dt);

	// variables for tracking time if "use_sim_time" is on
	double elapsed_time = 0.0;
	int nloops = 0;

	// ros simulation loop
	while (ros::ok()) {

		// publish the odometry message
		odom_msg.header.stamp = ros::Time::now();
		odom_msg.pose.pose.position.x += twist.linear.x*5.0f*dt;
		odom_pub.publish(odom_msg);
		odom_msg.header.seq++;

		
		if (nloops%10==0){
			// publish the point cloud at 10 Hz
			pc2.header.stamp = ros::Time::now();
			lidar_pub.publish(pc2);
			pc2.header.seq++;
		}
			
		// update and publish time if necessary
		if (use_sim_time ){
			ros::Time now(elapsed_time);
			rosgraph_msgs::Clock clock_msg;
			clock_msg.clock = now; 
      		clock_pub.publish(clock_msg);
			elapsed_time += dt;
		}
		else {
			rate.sleep();
		}
		
		ros::spinOnce();
		nloops++;
	} //while ros OK


	return 0;
}

