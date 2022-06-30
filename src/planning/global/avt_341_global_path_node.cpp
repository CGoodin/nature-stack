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
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
// local includes
#include "avt_341/avt_341_utils.h"
#include "avt_341/planning/global/astar.h"
#include "avt_341/visualization/visualization_factory.h"
avt_341::msg::Odometry odom;
bool odom_rcvd = false;
avt_341::msg::OccupancyGrid current_grid;
avt_341::msg::OccupancyGrid segmentation_grid;
avt_341::msg::Path current_waypoints;
bool waypoints_rcvd = false;

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom)
{
  odom = *rcv_odom;
  odom_rcvd = true;
}

void MapCallback(avt_341::msg::OccupancyGridPtr rcv_grid)
{
  current_grid = *rcv_grid;
}

void SegmentationMapCallback(avt_341::msg::OccupancyGridPtr rcv_grid){
    segmentation_grid = *rcv_grid;
}

void WaypointCallback(avt_341::msg::PathPtr rcv_waypoints)
{
  //std::cout << "Waypoints received!" << std::endl;
  // Brute force - overwrite the current global waypoints
  current_waypoints = *rcv_waypoints;
  waypoints_rcvd = true;

}

int main(int argc, char *argv[])
{
  auto n = avt_341::node::init_node(argc, argv, "avt_341_global_path_node");

  auto path_pub = n->create_publisher<avt_341::msg::Path>("avt_341/global_path", 10);
  auto waypoint_pub = n->create_publisher<avt_341::msg::Path>("avt_341/waypoints", 10);
  auto current_waypoint_pub = n->create_publisher<avt_341::msg::Int32>("avt_341/current_waypoint", 10);
  auto dist_to_current_waypoint_pub = n->create_publisher<avt_341::msg::Float64>("avt_341/distance_to_current_waypoint", 10);
  auto odometry_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
  auto map_sub = n->create_subscription<avt_341::msg::OccupancyGrid>("avt_341/occupancy_grid", 10, MapCallback);
  auto segmentation_map_sub = n->create_subscription<avt_341::msg::OccupancyGrid>("avt_341/segmentation_grid", 10, SegmentationMapCallback);
  auto waypoint_sub = n->create_subscription<avt_341::msg::Path>("avt_341/new_waypoints", 10, WaypointCallback);

  // ctg, 8-19-2021
  // the state values can be
  // -1 - startup (stopped and not shut down)
  // 0 - active 
  // 1 - bring to a smooth stop but do not shut down
  // 2 - bring to a smooth stop and shut down
  // 3 - bring to an immediate stop (hard braking) and shut down
  auto state_pub = n->create_publisher<avt_341::msg::Int32>("avt_341/state", 10);
  avt_341::msg::Int32 state;
  state.data = -1; // start up state

  float goal_dist, global_lookahead;
  std::vector<double> waypoints_x_list, waypoints_y_list;
  std::string display_type;

  std::vector<float> goal;
  goal.resize(2, 0.0f);

  n->get_parameter("~goal_dist", goal_dist, 3.0f);
  n->get_parameter("~display", display_type, avt_341::visualization::default_display);
  n->get_parameter("~global_lookahead", global_lookahead, 50.0f);
  n->get_parameter("/waypoints_x", waypoints_x_list, std::vector<double>(0));
  n->get_parameter("/waypoints_y", waypoints_y_list, std::vector<double>(0));
  
  int shutdown_behavior = 1;
  n->get_parameter("~shutdown_behavior", shutdown_behavior, 1);
  if (shutdown_behavior>3 || shutdown_behavior<1)shutdown_behavior = 1;

  if (waypoints_x_list.size() != waypoints_y_list.size())
  {
    std::cerr << "WARNING: " << waypoints_x_list.size() << " X COORDINATES WERE PROVIDED FOR " << waypoints_y_list.size() << " Y COORDINATES." << std::endl;
  }
  if (waypoints_x_list.size() == 0 || waypoints_y_list.size() == 0)
  {
    std::cerr << "WARNING: NO WAYPOINTS WERE LISTED IN /waypoints_x OR /waypoints_y." << std::endl;
    //return 2;
  }

  int num_waypoints = std::min(waypoints_x_list.size(), waypoints_y_list.size());

  // Initialize current waypoints with the data from the waypoint yaml params
  if (num_waypoints > 0)
  {
    //nav_msgs::Path loaded_waypoints;
    current_waypoints.poses.clear();
    //current_waypoints.header.frame_id = "odom";
    current_waypoints.header.frame_id = "map";
    for (int32_t i=0;i<num_waypoints;i++){
      avt_341::msg::PoseStamped pose;
      pose.pose.position.x = static_cast<float>(waypoints_x_list[i]);
      pose.pose.position.y = static_cast<float>(waypoints_y_list[i]);
      pose.pose.position.z = 0.0f;
      pose.pose.orientation.w = 1.0f;
      pose.pose.orientation.x = 0.0f;
      pose.pose.orientation.y = 0.0f;
      pose.pose.orientation.z = 0.0f;
      current_waypoints.poses.push_back(pose);
    }
      // Initialize goal to first waypoint
    goal[0] = waypoints_x_list[0];
    goal[1] = waypoints_y_list[0];
    state.data = 0; // go active
    state_pub->publish(state);
  }

  auto visualizer = avt_341::visualization::create_visualizer(display_type);
  avt_341::planning::Astar astar_planner(visualizer);

  avt_341::node::Rate r(20.0f); // Hz
  bool shutdown_condition = false;
  int nl = 0;
  int current_waypoint = 0;
  int shutdown_count = 0;
  bool waypoints_change_once = true;
  //while (avt_341::node::ok() && !goal_reached){
  while (avt_341::node::ok()){
    state_pub->publish(state);
    if (waypoints_rcvd && waypoints_change_once) {
      // process a new set of waypoints
      // TODO: find closest point along path -  we probably don't want to reverse back to start point if we're past it.
      current_waypoint = 0;
      goal[0] = current_waypoints.poses[current_waypoint].pose.position.x;
      goal[1] = current_waypoints.poses[current_waypoint].pose.position.y;
      std::cout << "New waypoints! Updated goal " << goal[0] << ", " << goal[1] << std::endl;
      //waypoints_rcvd = false;
      waypoints_change_once = false;
      state.data = 0;  // go active
      state_pub->publish(state);
    }

    if (odom_rcvd && state.data != -1){ // data received and not in startup mode
      std::vector<float> pos;
      pos.push_back(odom.pose.pose.position.x);
      pos.push_back(odom.pose.pose.position.y);

      std::vector<std::vector<float>> path = astar_planner.PlanPath(&current_grid, &segmentation_grid, goal, pos);

      avt_341::msg::Path ros_path;
      //ros_path.header.frame_id = "odom";
      ros_path.header.frame_id = "map";
      ros_path.poses.clear();
      for (int32_t i = 0; i < path.size(); i++){
        avt_341::msg::PoseStamped pose;
        pose.pose.position.x = static_cast<float>(path[i][0]);
        pose.pose.position.y = static_cast<float>(path[i][1]);
        pose.pose.position.z = 0.0f;
        pose.pose.orientation.w = 1.0f;
        pose.pose.orientation.x = 0.0f;
        pose.pose.orientation.y = 0.0f;
        pose.pose.orientation.z = 0.0f;
        ros_path.poses.push_back(pose);
      }
      // ctg 8/19/21
      // if not on the last waypoint, add a straight path to the next waypoint to the global path
      // this helps the local planner make smooth transitions between waypoints
      if (ros_path.poses.size()>1) {
        int cp =current_waypoint;
        while (cp<current_waypoints.poses.size()-1){
          avt_341::msg::PoseStamped pose;
          pose.pose.position.x = static_cast<float>(current_waypoints.poses[cp+1].pose.position.x);
          pose.pose.position.y = static_cast<float>(current_waypoints.poses[cp+1].pose.position.y);
          pose.pose.position.z = 0.0f;
          pose.pose.orientation.w = 1.0f;
          pose.pose.orientation.x = 0.0f;
          pose.pose.orientation.y = 0.0f;
          pose.pose.orientation.z = 0.0f;
          ros_path.poses.push_back(pose);
          cp++;
        }
      }

      ros_path.header.stamp = n->get_stamp();
      avt_341::node::set_seq(ros_path.header, nl);

      for (int i = 0; i < ros_path.poses.size(); i++){
        ros_path.poses[i].header = ros_path.header;
      }

      path_pub->publish(ros_path);
      waypoint_pub->publish(current_waypoints);


      // check the progression along the path
      float dx = goal[0] - odom.pose.pose.position.x;
      float dy = goal[1] - odom.pose.pose.position.y;
      double d = sqrt(dx * dx + dy * dy);
      avt_341::msg::Float64 dist_to_goal;
      dist_to_goal.data = d;
      avt_341::msg::Int32 curr_wp;
      curr_wp.data = current_waypoint;
      current_waypoint_pub->publish(curr_wp);
      dist_to_current_waypoint_pub->publish(dist_to_goal);
      if (nl % 20 == 0){ //update every second
        std::cout << "Distance to goal " << current_waypoint<<" = " << d << std::endl;
      }
      if (current_waypoint == current_waypoints.poses.size() - 1){  // last waypoint
        if (d<goal_dist || shutdown_condition){   // reached the goal
          shutdown_condition = true;
          state.data = shutdown_behavior; // request shutdown behavior
          state_pub->publish(state);
          shutdown_count++;
          if (shutdown_count>10) break;
        }
      }
      else{     // intermediate waypoint
        if (d<goal_dist){   // reached the waypoint
          current_waypoint++;
          goal[0] = current_waypoints.poses[current_waypoint].pose.position.x;
          goal[1] = current_waypoints.poses[current_waypoint].pose.position.y;
        }
        state.data = 0;         // request active behavior
        state_pub->publish(state);
      }
    } // if odom_recvd
    //else if(state.data != -1){  // not in startup
    //  state.data = 1;       // request smooth stop but don't shutdown (waiting for odom data)
    //  state_pub->publish(state);
    //}

    n->spin_some();
    r.sleep();
    nl++;
  }

  return 0;
}
