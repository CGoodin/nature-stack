/**
 * \file path_manager_node.cpp
 * 
 * ROS node that demonstrates publishing an updated path. Param wait_period controls when the new waypoints are published. Default is 30 seconds which works with the mavs_example.
 * 
 * \author Daniel Carruth
 *
 * \contact dwc2@cavs.msstate.edu
 * 
 * \date 8/11/2020
 */

// ros includes
#include "nature/node/ros_types.h"
#include "nature/node/node_proxy.h"
// local includes
#include "nature/nature_utils.h"

nature::msg::Path new_path;

int main(int argc, char **argv)
{
    auto n = nature::node::init_node(argc,argv,"path_manager");

    auto path_pub = n->create_publisher<nature::msg::Path>("nature/new_waypoints", 10);
    
    nature::node::Rate loop_rate(10);

    float wait_period;
    n->get_parameter("~wait_period", wait_period, 30.0f);

    auto start_time = n->get_stamp();
    nature::node::Duration wait_duration = nature::node::make_duration(wait_period);

    int count = 0;
    int msg_sent = 0;
    while(nature::node::ok())
    {
        if(!msg_sent && n->get_stamp() > start_time + wait_duration)
        {
            std::vector<std::vector<float>> path = {{0, 10}, {10, 20}, {10, 50}};

            nature::msg::Path ros_path;
            ros_path.header.frame_id = "odom";
            ros_path.poses.clear();
            for (int32_t i = 0; i < path.size(); i++){
                nature::msg::PoseStamped pose;
                pose.pose.position.x = static_cast<float>(path[i][0]);
                pose.pose.position.y = static_cast<float>(path[i][1]);
                pose.pose.position.z = 0.0f;
                pose.pose.orientation.w = 1.0f;
                pose.pose.orientation.x = 0.0f;
                pose.pose.orientation.y = 0.0f;
                pose.pose.orientation.z = 0.0f;
                ros_path.poses.push_back(pose);
            } 

            ros_path.header.stamp = n->get_stamp();
            nature::node::set_seq(ros_path.header, count);

            for (int i = 0; i < ros_path.poses.size(); i++){
                ros_path.poses[i].header = ros_path.header;
            }

            path_pub->publish(ros_path);
            count++;
            msg_sent = 1;
        }
        n->spin_some();
        loop_rate.sleep();
    }
}
