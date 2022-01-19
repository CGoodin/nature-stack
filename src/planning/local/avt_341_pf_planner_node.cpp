/**
 * \file avt_341_pf_planner_node.cpp
 * Plan a local trajectory using the potential field planner
 * 
 * \author Chris Goodin
 *
 * \contact cgoodin@cavs.msstate.edu
 * 
 * \date 1/19/2022
 */
// ROS includes
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
// avt_341 includes
#include "avt_341/planning/local/pf_planner.h"
#include "avt_341/visualization/visualization_factory.h"

avt_341::msg::Odometry odom;
avt_341::msg::OccupancyGrid grid;
avt_341::msg::OccupancyGrid segmentation_grid;
avt_341::msg::Path global_path;
avt_341::msg::Path waypoints;
bool odom_rcvd = false;
bool new_grid_rcvd = false;
bool new_seg_grid_rcvd = false;

void OdometryCallback(avt_341::msg::OdometryPtr rcv_odom){
  odom = *rcv_odom;
  odom_rcvd = true;
}

void GridCallback(avt_341::msg::OccupancyGridPtr rcv_grid){
  grid = *rcv_grid;
  new_grid_rcvd = true;
}

void SegmentationGridCallback(avt_341::msg::OccupancyGridPtr rcv_grid){
    segmentation_grid = *rcv_grid;
    new_seg_grid_rcvd = true;
}

void PathCallback(avt_341::msg::PathPtr rcv_path){
  global_path = *rcv_path;
}

void WaypointCallback(avt_341::msg::PathPtr wp_path){
  waypoints = *wp_path;
}

int main(int argc, char *argv[]){

  auto n = avt_341::node::init_node(argc, argv, "avt_341_pf_planner_node");

  // Create publishers and subscribers
  auto path_pub = n->create_publisher<avt_341::msg::Path>("avt_341/local_path", 10);
  auto odometry_sub = n->create_subscription<avt_341::msg::Odometry>("avt_341/odometry", 10, OdometryCallback);
  auto grid_sub = n->create_subscription<avt_341::msg::OccupancyGrid>("avt_341/occupancy_grid", 10, GridCallback);
  auto segmentation_grid_sub = n->create_subscription<avt_341::msg::OccupancyGrid>("avt_341/segmentation_grid", 10, SegmentationGridCallback);
  auto path_sub = n->create_subscription<avt_341::msg::Path>("avt_341/global_path", 10, PathCallback);
  auto wp_sub = n->create_subscription<avt_341::msg::Path>("avt_341/waypoints", 10, WaypointCallback);

  // planner params
  float kp, eta, cutoff_dist, rate;
  bool use_global_path;
  n->get_parameter("~kp", kp, 5.0f);
  n->get_parameter("~eta", eta, 100.0f);
  n->get_parameter("~cutoff_dist", cutoff_dist, 20.0f);
  n->get_parameter("~use_global_path", use_global_path, false);
  n->get_parameter("~rate", rate, 50.0f);

  avt_341::planning::PfPlanner planner;
  planner.SetEta(eta);
  planner.SetKp(kp);
  planner.SetCutoffDistance(cutoff_dist);

  unsigned int loop_count = 0;
  float dt = 1.0f / rate;
  float elapsed_time = 0.0f;
  avt_341::node::Rate rosrate(rate);
  while (avt_341::node::ok()){
    double start_secs = n->get_now_seconds();
    if (global_path.poses.size() > 0 && odom_rcvd && grid.data.size() > 0){

      float gx, gy;
      if (use_global_path){
        gx = global_path.poses.back().pose.position.x;
        gy = global_path.poses.back().pose.position.y;
      }
      else{
        gx = waypoints.poses.back().pose.position.x;
        gy = waypoints.poses.back().pose.position.y;
      }

      planner.SetGoal(gx, gy);

      avt_341::msg::Path local_path = planner.Plan(grid, odom);

      local_path.header.frame_id = "map";
      local_path.header.stamp = n->get_stamp();
      avt_341::node::set_seq(local_path.header, loop_count);
      path_pub->publish(local_path);
      /*}
      else {
        avt_341::msg::Path local_path;
        avt_341::msg::PoseStamped pose;
        pose.pose = odom.pose.pose;
        local_path.poses.push_back(pose);
        local_path.header.frame_id = "map";
        local_path.header.stamp = n->get_stamp();
        avt_341::node::set_seq(local_path.header, loop_count);
        path_pub->publish(local_path);
      }*/

      odom_rcvd = false;
    }
    else {
      if (global_path.poses.size() <= 0){
        //std::cout << "Local planner did not run because global path not recieved " << std::endl;
      }
      else if (!odom_rcvd){
        //std::cout << "Local planner did not run because vehicle odometry not recieved." << std::endl;
      }
      else if (grid.data.size() <= 0){
        //std::cout << "Local planner did not run because occupancy grid not recieved." << std::endl;
      }
    }
    new_grid_rcvd = false;
    new_seg_grid_rcvd = false;
    loop_count++;
    double end_secs = n->get_now_seconds();
    if ((end_secs - start_secs) > 2.5 * dt){
      //std::cout << "WARNING: TANG PLANNER TOOK " << (end_secs - start_secs) << " TO COMPLETE. REQUESTED UPDATE SPEED IS " << dt << std::endl;
    }

    n->spin_some();
    rosrate.sleep();
  }

  return 0;
}
