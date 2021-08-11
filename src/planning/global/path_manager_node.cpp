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
#include "ros/ros.h"
#include <nav_msgs/Path.h>
// local includes
#include "avt_341/avt_341_utils.h"

nav_msgs::Path new_path;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_manager");
    ros::NodeHandle n;

    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("avt_341/new_waypoints", 10);
    
    ros::Rate loop_rate(10);

    float wait_period = 30.0f;
    if (ros::param::has("~wait_period")){
        ros::param::get("~wait_period", wait_period);
    }

    ros::Time start_time = ros::Time::now();
    ros::Duration wait_duration = ros::Duration(wait_period);

    int count = 0;
    int msg_sent = 0;
    while(ros::ok())
    {
        if(!msg_sent && ros::Time::now() > start_time + wait_duration)
        {
            std::vector<std::vector<float>> path = {{0, 10}, {10, 20}, {10, 50}};

            nav_msgs::Path ros_path;
            ros_path.header.frame_id = "odom";
            ros_path.poses.clear();
            for (int32_t i = 0; i < path.size(); i++){
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = static_cast<float>(path[i][0]);
                pose.pose.position.y = static_cast<float>(path[i][1]);
                pose.pose.position.z = 0.0f;
                pose.pose.orientation.w = 1.0f;
                pose.pose.orientation.x = 0.0f;
                pose.pose.orientation.y = 0.0f;
                pose.pose.orientation.z = 0.0f;
                ros_path.poses.push_back(pose);
            } 

            ros_path.header.stamp = ros::Time::now();
            ros_path.header.seq = count;

            for (int i = 0; i < ros_path.poses.size(); i++){
                ros_path.poses[i].header = ros_path.header;
            }

            path_pub.publish(ros_path);
            count++;
            msg_sent = 1;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
