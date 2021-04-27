/**
 * \file avt_341_global_path_node.cpp
 * 
 * ROS Node that publishes a user defined path as a global path.
 * 
 * \author Chris Goodin
 *
 * \contact cgoodin@cavs.msstate.edu
 * 
 * \date 9/1/2020
 */

// ros includes
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include "nav_msgs/Odometry.h"
// local includes
#include "avt_341/avt_341_utils.h"
#include "avt_341/planning/global/astar.h"

nav_msgs::Odometry odom;
bool odom_rcvd = false;
nav_msgs::OccupancyGrid current_grid;

void OdometryCallback(const nav_msgs::Odometry::ConstPtr &rcv_odom)
{
  odom = *rcv_odom;
  odom_rcvd = true;
}

void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &rcv_grid)
{
  current_grid = *rcv_grid;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "avt_341_global_path_node");
  ros::NodeHandle n;

  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("avt_341/global_path", 10);
  ros::Publisher waypoint_pub = n.advertise<nav_msgs::Path>("avt_341/waypoints", 10);
  ros::Subscriber odometry_sub = n.subscribe("avt_341/odometry", 10, OdometryCallback);
  ros::Subscriber map_sub = n.subscribe("avt_341/occupancy_grid", 10, MapCallback);

  float goal_dist = 3.0f;
  if (ros::param::has("~goal_dist")){
    ros::param::get("~goal_dist", goal_dist);
  }
  float global_lookahead = 50.0f;
  if (ros::param::has("~global_lookahead")){
    ros::param::get("~global_lookahead", global_lookahead);
  }

  std::vector<double> waypoints_x_list, waypoints_y_list;
  if (!(n.hasParam("/waypoints_x") && "/waypoints_y"))
  {
    std::cerr << "ERROR: NO WAYPOINTS WERE PROVIDED, EXITING." << std::endl;
    return 1;
  }
  n.getParam("/waypoints_x", waypoints_x_list);
  n.getParam("/waypoints_y", waypoints_y_list);

  if (waypoints_x_list.size() != waypoints_y_list.size())
  {
    std::cerr << "WARNING: " << waypoints_x_list.size() << " X COORDINATES WERE PROVIDED FOR " << waypoints_y_list.size() << " Y COORDINATES." << std::endl;
  }
  if (waypoints_x_list.size() == 0 || waypoints_y_list.size() == 0)
  {
    std::cerr << "ERROR: NO WAYPOINTS WERE LISTED IN /waypoints_x OR /waypoints_y, EXITING." << std::endl;
    return 2;
  }

  avt_341::planning::Astar astar_planner;

  int num_waypoints = std::min(waypoints_x_list.size(), waypoints_y_list.size());

  nav_msgs::Path loaded_waypoints;
  loaded_waypoints.header.frame_id = "odom"; 
  loaded_waypoints.poses.clear();
  for (int32_t i=0;i<num_waypoints;i++){
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = static_cast<float>(waypoints_x_list[i]);
    pose.pose.position.y = static_cast<float>(waypoints_y_list[i]);
    pose.pose.position.z = 0.0f;
    pose.pose.orientation.w = 1.0f;
    pose.pose.orientation.x = 0.0f;
    pose.pose.orientation.y = 0.0f;
    pose.pose.orientation.z = 0.0f;
    loaded_waypoints.poses.push_back(pose);
  }

  std::vector<float> goal;
  goal.resize(2, 0.0f);
  goal[0] = waypoints_x_list[0];
  goal[1] = waypoints_y_list[0];  


  ros::Rate r(10.0f); // Hz
  bool goal_reached = false;
  int nl = 0;
  int current_waypoint = 0;
  while (ros::ok() && !goal_reached){

    if (odom_rcvd){
      std::vector<float> pos;
      pos.push_back(odom.pose.pose.position.x);
      pos.push_back(odom.pose.pose.position.y);

      std::vector<std::vector<float>> path = astar_planner.PlanPath(&current_grid, goal, pos);

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
      ros_path.header.seq = nl;

      for (int i = 0; i < ros_path.poses.size(); i++){
        ros_path.poses[i].header = ros_path.header;
      }

      path_pub.publish(ros_path);
      waypoint_pub.publish(loaded_waypoints);

      float dx = goal[0] - odom.pose.pose.position.x;
      float dy = goal[1] - odom.pose.pose.position.y;
      double d = sqrt(dx * dx + dy * dy);
      if (nl % 100 == 0){ //update every 2 seconds
        std::cout << "Distance to goal = " << d << std::endl;
      }
      //if (d < goal_dist){
      if (d < global_lookahead){
        if (current_waypoint == waypoints_x_list.size() - 1){
          if ( d<goal_dist){
            goal_reached = true;
          }
        }
        else{
          current_waypoint++;
          goal[0] = waypoints_x_list[current_waypoint];
          goal[1] = waypoints_y_list[current_waypoint];
        }
      }
    }

    ros::spinOnce();
    r.sleep();
    nl++;
  }

  return 0;
}
