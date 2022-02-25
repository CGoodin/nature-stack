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
	auto mpc_state_pub = n->create_publisher<avt_341::msg::Float64MultiArray>("avt_341/veh",1);


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
  avt_341::node::set_seq(odom_msg.header, 0);
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
	  avt_341::utils::vec3(50.0, 0.0, 0.0),
    avt_341::utils::vec3(15.1, 7.8, 5.0),
    avt_341::utils::vec3(15.1, 7.8, 1.0),
    avt_341::utils::vec3(14.5, 8.5, 7.0),
    avt_341::utils::vec3(14.5, 8.5, 1.0),
    avt_341::utils::vec3(14.6, 8.2, 4.5),
    avt_341::utils::vec3(14.6, 8.2, 1.5),
    avt_341::utils::vec3(15.1, -7.8, 0.0),
    avt_341::utils::vec3(14.5, -8.5, 0.0),
    avt_341::utils::vec3(14.6, -8.2, 0.1)
	};
    std::vector<int> seg_values = {
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            1,
            1,
            1
    };
	std::vector<double> veh_data = {0.0, -50.0, 1.8, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	avt_341::msg::Float64MultiArray mpc_data_msg;
	mpc_data_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	mpc_data_msg.layout.dim[0].size = veh_data.size();
	mpc_data_msg.layout.dim[0].stride = 1;
	mpc_data_msg.layout.dim[0].label = "x";

  avt_341::perception::PointCloudGenerator::toROSMsg(points, seg_values,pc2);
  pc2.header.frame_id = "odom";
  avt_341::node::set_seq(pc2.header, 0);

	//odometry published at 100 Hz, point clout at 10 Hz
	double dt = 0.01;
	avt_341::node::Rate rate(1.0/dt);

	// variables for tracking time if "use_sim_time" is on
	double elapsed_time = 0.0;
	int nloops = 0;
	float desired_speed = 5.0f;
	// ros simulation loop
	while (avt_341::node::ok()) {

		// publish the odometry message
		odom_msg.header.stamp = n->get_stamp();
		odom_msg.pose.pose.position.x += twist.linear.x*desired_speed*dt;
		odom_msg.twist.twist.linear.x = desired_speed*twist.linear.x;
		odom_pub->publish(odom_msg);
		veh_data[1] = odom_msg.pose.pose.position.x;
		veh_data[2] = odom_msg.pose.pose.position.y;
		veh_data[3] = odom_msg.twist.twist.linear.x;
		veh_data[4] = odom_msg.twist.twist.linear.y;
		mpc_data_msg.data.clear();
		mpc_data_msg.data.insert(mpc_data_msg.data.end(), veh_data.begin(), veh_data.end());
		mpc_state_pub->publish(mpc_data_msg);
		avt_341::node::inc_seq(odom_msg.header);

		
		if (nloops%10==0){
			// publish the point cloud at 10 Hz
			pc2.header.stamp = n->get_stamp();
			lidar_pub->publish(pc2);
			avt_341::node::inc_seq(pc2.header);
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

