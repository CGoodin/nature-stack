//ros includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "avt_341/node/clock_publisher.h"
// point cloud includes
#include "avt_341/perception/point_cloud_generator.h"

avt_341::msg::Twist twist;
void TwistCallback(avt_341::msg::TwistPtr rcv_msg){
	twist.linear.x = rcv_msg->linear.x; // throttle
	twist.linear.y = rcv_msg->linear.y; // braking
	twist.angular.z = rcv_msg->angular.z; // steering
}

int main(int argc, char **argv){

  auto n = avt_341::node::init_node(argc,argv,"avt_341_simulation_test_node");

	auto twist_sub = n->create_subscription<avt_341::msg::Twist>("avt_341/cmd_vel",1, TwistCallback);

	auto lidar_pub = n->create_publisher<avt_341::msg::PointCloud2>("avt_341/points",1);
	auto odom_pub = n->create_publisher<avt_341::msg::Odometry>("avt_341/odometry",1);


	// determine if sim time will be used
	bool use_sim_time;
	n->get_parameter("use_sim_time", use_sim_time, false);
	std::shared_ptr<avt_341::node::ClockPublisher> clock_pub;
	if (use_sim_time){
    clock_pub = avt_341::node::ClockPublisher::make_shared("clock", 1, n);
	}

	// create and populate the odometry message that will be published
	avt_341::msg::Odometry odom_msg;
	odom_msg.header.frame_id = "odom";
	odom_msg.header.seq = 0;
	odom_msg.pose.pose.position.x = -55.0;
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
  avt_341::msg::PointCloud2 pc2;
	std::vector<avt_341::utils::vec3> points {
	  avt_341::utils::vec3(50.0, 0.0, 0.0)
	};
  avt_341::perception::PointCloudGenerator::toROSMsg(points, pc2);
  pc2.header.frame_id = "odom";
  pc2.header.seq = 0;

	//odometry published at 100 Hz, point clout at 10 Hz
	double dt = 0.01;
	avt_341::node::Rate rate(1.0/dt);

	// variables for tracking time if "use_sim_time" is on
	double elapsed_time = 0.0;
	int nloops = 0;

	// ros simulation loop
	while (avt_341::node::ok()) {

		// publish the odometry message
		odom_msg.header.stamp = n->get_stamp();
		odom_msg.pose.pose.position.x += twist.linear.x*5.0f*dt;
		odom_pub->publish(odom_msg);
		odom_msg.header.seq++;

		
		if (nloops%10==0){
			// publish the point cloud at 10 Hz
			pc2.header.stamp = n->get_stamp();
			lidar_pub->publish(pc2);
			pc2.header.seq++;
		}
			
		// update and publish time if necessary
		if (use_sim_time ){
      clock_pub->publish(elapsed_time);
			elapsed_time += dt;
		}
		else {
			rate.sleep();
		}
		
		n->spin_some();
		nloops++;
	} //while ros OK


	return 0;
}

