//ros includes
#include "nature/node/ros_types.h"
#include "nature/node/node_proxy.h"
#include "nature/node/clock_publisher.h"
// point cloud includes
#include "nature/perception/point_cloud_generator.h"

nature::msg::Twist twist;
void TwistCallback(nature::msg::TwistPtr rcv_msg){
	twist.linear.x = rcv_msg->linear.x; // throttle
	twist.linear.y = rcv_msg->linear.y; // braking
	twist.angular.z = rcv_msg->angular.z; // steering
}

int main(int argc, char **argv){

  auto n = nature::node::init_node(argc,argv,"nature_simulation_test_node");

	auto twist_sub = n->create_subscription<nature::msg::Twist>("nature/cmd_vel",1, TwistCallback);

	auto lidar_pub = n->create_publisher<nature::msg::PointCloud2>("nature/points",1);
	auto odom_pub = n->create_publisher<nature::msg::Odometry>("nature/odometry",1);
	auto mpc_state_pub = n->create_publisher<nature::msg::Float64MultiArray>("nature/veh",1);


	// determine if sim time will be used
	bool use_sim_time;
	n->get_parameter("use_sim_time", use_sim_time, false);
	std::shared_ptr<nature::node::ClockPublisher> clock_pub;
	if (use_sim_time){
    clock_pub = nature::node::ClockPublisher::make_shared("clock", 1, n);
	}

	// create and populate the odometry message that will be published
	nature::msg::Odometry odom_msg;
	odom_msg.header.frame_id = "odom";
  nature::node::set_seq(odom_msg.header, 0);
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
  nature::msg::PointCloud2 pc2;
	std::vector<nature::utils::vec3> points {
	  nature::utils::vec3(50.0, 0.0, 0.0),
    nature::utils::vec3(15.1, 7.8, 5.0),
    nature::utils::vec3(15.1, 7.8, 1.0),
    nature::utils::vec3(14.5, 8.5, 7.0),
    nature::utils::vec3(14.5, 8.5, 1.0),
    nature::utils::vec3(14.6, 8.2, 4.5),
    nature::utils::vec3(14.6, 8.2, 1.5),
    nature::utils::vec3(15.1, -7.8, 0.0),
    nature::utils::vec3(14.5, -8.5, 0.0),
    nature::utils::vec3(14.6, -8.2, 0.1)
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
	nature::msg::Float64MultiArray mpc_data_msg;
	//mpc_data_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	mpc_data_msg.layout.dim.push_back(nature::msg::MultiArrayDimension());
	mpc_data_msg.layout.dim[0].size = veh_data.size();
	mpc_data_msg.layout.dim[0].stride = 1;
	mpc_data_msg.layout.dim[0].label = "x";

  nature::perception::PointCloudGenerator::toROSMsg(points, seg_values,pc2);
  pc2.header.frame_id = "odom";
  nature::node::set_seq(pc2.header, 0);

	//odometry published at 100 Hz, point clout at 10 Hz
	double dt = 0.01;
	nature::node::Rate rate(1.0/dt);

	// variables for tracking time if "use_sim_time" is on
	double elapsed_time = 0.0;
	int nloops = 0;
	float desired_speed = 5.0f;
	// ros simulation loop
	while (nature::node::ok()) {

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
		nature::node::inc_seq(odom_msg.header);

		
		if (nloops%10==0){
			// publish the point cloud at 10 Hz
			pc2.header.stamp = n->get_stamp();
			lidar_pub->publish(pc2);
			nature::node::inc_seq(pc2.header);
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

