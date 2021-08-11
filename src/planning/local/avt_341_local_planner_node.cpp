/**
 * \file avt_341_planner_node.cpp
 * Plan a local trajectory using a global path.
 * 
 * \author Chris Goodin
 *
 * \contact cgoodin@cavs.msstate.edu
 * 
 * \date 8/31/2020
 */
// ROS includes
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "rosgraph_msgs/Clock.h"
#include <tf/transform_datatypes.h>
// avt_341 includes
#include "avt_341/planning/local/spline_planner.h"
#include "avt_341/planning/local/spline_plotter.h"

nav_msgs::Odometry odom;
nav_msgs::OccupancyGrid grid;
nav_msgs::Path global_path;
nav_msgs::Path waypoints;
bool odom_rcvd = false;
bool new_grid_rcvd = false;

void OdometryCallback(const nav_msgs::Odometry::ConstPtr &rcv_odom){
  odom = *rcv_odom;
  odom_rcvd = true;
}

void GridCallback(const nav_msgs::OccupancyGrid::ConstPtr &rcv_grid){
  grid = *rcv_grid;
  new_grid_rcvd = true;
}

void PathCallback(const nav_msgs::Path::ConstPtr &rcv_path){
  global_path = *rcv_path;
}

void WaypointCallback(const nav_msgs::Path::ConstPtr &wp_path){
  waypoints = *wp_path;
}

int main(int argc, char *argv[]){
  ros::init(argc, argv, "avt_341_planner_node");
  ros::NodeHandle n;

  // Create publishers and subscribers
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("avt_341/local_path", 10);
  ros::Subscriber odometry_sub = n.subscribe("avt_341/odometry", 10, OdometryCallback);
  ros::Subscriber grid_sub = n.subscribe("avt_341/occupancy_grid", 10, GridCallback);
  ros::Subscriber path_sub = n.subscribe("avt_341/global_path", 10, PathCallback);
  ros::Subscriber wp_sub = n.subscribe("avt_341/waypoints", 10, WaypointCallback);
  ros::Publisher clock_pub;

  avt_341::planning::Planner planner;
  // planner params
  float path_look_ahead = 15.0f;
  float vehicle_width = 3.0f;
  int num_paths = 31;
  float max_steer_angle = 0.43;
  float output_path_step = 0.5f;
  float path_int_step = 0.25f;
   int dilation_factor = 0;
  if (ros::param::has("~path_look_ahead")){
    ros::param::get("~path_look_ahead", path_look_ahead);
  }
  if (ros::param::has("~vehicle_width")){
    ros::param::get("~vehicle_width", vehicle_width);
  }
  if (ros::param::has("~num_paths")){
    ros::param::get("~num_paths", num_paths);
  }
  if (ros::param::has("~max_steer_angle")){
    ros::param::get("~max_steer_angle", max_steer_angle);
  }
  if (ros::param::has("~output_path_step")){
    ros::param::get("~output_path_step", output_path_step);
  }
  if (ros::param::has("~path_integration_step")){
    ros::param::get("~path_integration_step", path_int_step);
    planner.SetArcLengthIntegrationStep(path_int_step);
  }
  if (ros::param::has("~dilation_factor")){
    ros::param::get("~dilation_factor", dilation_factor);
  }
  if (ros::param::has("~w_c")){
    float w_c;
    ros::param::get("~w_c", w_c);
    planner.SetComfortabilityWeight(w_c);
  }
  if (ros::param::has("~w_d")){
    float w_d;
    ros::param::get("~w_d", w_d);
    planner.SetDynamicSafetyWeight(w_d);
  }
  if (ros::param::has("~w_s")){
    float w_s;
    ros::param::get("~w_s", w_s);
    planner.SetStaticSafetyWeight(w_s);
  }
  if (ros::param::has("~w_r")){
    float w_r;
    ros::param::get("~w_r", w_r);
    planner.SetPathAdherenceWeight(w_r);
  }
  float rate = 50.0f; //Hz
  if (ros::param::has("~rate")){
    ros::param::get("~rate", rate);
  }
  bool trim_path = false;
  if (ros::param::has("~trim_path")){
    ros::param::get("~trim_path", trim_path);
  }

  bool display = false;
  if (ros::param::has("~display")){
    ros::param::get("~display", display);
  }

  avt_341::planning::Plotter plotter;

  unsigned int loop_count = 0;
  float dt = 1.0f / rate;
  float elapsed_time = 0.0f;
  ros::Rate rosrate(rate);
  while (ros::ok()){
    double start_secs = ros::WallTime::now().toSec();
    if (global_path.poses.size() > 0 && odom_rcvd && grid.data.size() > 0){
      std::vector<avt_341::utils::vec2> path_points;
      for (int i = 0; i < global_path.poses.size(); i++){
        avt_341::utils::vec2 point(global_path.poses[i].pose.position.x, global_path.poses[i].pose.position.y);
        path_points.push_back(point);
      }
      avt_341::planning::Path path;
      if (trim_path ){
        avt_341::utils::vec2 current_pos(odom.pose.pose.position.x, odom.pose.pose.position.y);
        path.Init(path_points, current_pos, 1.5f * path_look_ahead);
      }
      else{
        path.Init(path_points);
      }
      //path.FixBeginning(odom.pose.pose.position.x, odom.pose.pose.position.y);

      std::vector<avt_341::utils::vec2> culled_points = path.GetPoints();
      float s_max = path.GetTotalLength();
      
      avt_341::utils::vec2 srho = path.ToSRho(odom.pose.pose.position.x, odom.pose.pose.position.y);
      float s = srho.x;
      float rho_start = srho.y;
      avt_341::utils::vec2 pconv = path.ToCartesian(s,rho_start);

      float s_lookahead = std::min(path_look_ahead, s_max - s);
      float theta = avt_341::utils::GetHeadingFromOrientation(odom.pose.pose.orientation);
      avt_341::planning::CurveInfo ci = path.GetCurvatureAndAngle(s);

      planner.GeneratePaths(num_paths, s, rho_start, theta - ci.theta, s_lookahead, max_steer_angle, vehicle_width);
      planner.SetCenterline(path);

      // calculate bounds around the vehicle to limit grid dilation to space 10m behind and path_look_ahead distance in front of the vehicle
      float veh_heading_x = cos(theta);
      float veh_heading_y = sin(theta);
      float veh_left_offset_x = odom.pose.pose.position.x + (-veh_heading_y * (path_look_ahead/2));
      float veh_left_offset_y = odom.pose.pose.position.y + (veh_heading_x * (path_look_ahead/2));
      float veh_right_offset_x = odom.pose.pose.position.x + (-veh_heading_y * -(path_look_ahead/2));
      float veh_right_offset_y = odom.pose.pose.position.y + (veh_heading_x * -(path_look_ahead/2));
      float lf_bounds_x = veh_left_offset_x + (veh_heading_x * path_look_ahead);
      float lf_bounds_y = veh_left_offset_y + (veh_heading_y * path_look_ahead);
      float rf_bounds_x = veh_right_offset_x + (veh_heading_x * path_look_ahead);
      float rf_bounds_y = veh_right_offset_y + (veh_heading_y * path_look_ahead);
      float lr_bounds_x = veh_left_offset_x + (veh_heading_x * -10);
      float lr_bounds_y = veh_left_offset_y + (veh_heading_y * -10);
      float rr_bounds_x = veh_right_offset_x + (veh_heading_x * -10);
      float rr_bounds_y = veh_right_offset_y + (veh_heading_y * -10);
      float llx = std::min({lf_bounds_x, rf_bounds_x, lr_bounds_x, rr_bounds_x});
      float lly = std::min({lf_bounds_y, rf_bounds_y, lr_bounds_y, rr_bounds_y});
      float urx = std::max({lf_bounds_x, rf_bounds_x, lr_bounds_x, rr_bounds_x});
      float ury = std::max({lf_bounds_y, rf_bounds_y, lr_bounds_y, rr_bounds_y});

 
      if (new_grid_rcvd) planner.DilateGrid(grid, dilation_factor, llx, lly, urx, ury);
      // Note: if grid size gets large, DilateGrid can take a significant amount of time

      // most of the calculation time spent on this function call
      bool path_found = planner.CalculateCandidateCosts(grid, odom);
      if (display){
        plotter.AddMap(grid);
        plotter.SetPath(culled_points);
        plotter.AddWaypoints(waypoints);
        std::vector<avt_341::planning::Candidate> paths = planner.GetCandidates();
        plotter.AddCurves(paths);
        plotter.Display();
      }

      if (path_found){
        float ds = output_path_step;
        nav_msgs::Path local_path;
        avt_341::planning::Candidate best = planner.GetBestPath();
        float s0 = best.GetS0() + ds;
        float s_max = s0 + best.GetMaxLength() - ds;
        while (s0 < s_max){
          float rho0 = best.At(s0 - best.GetS0());
          avt_341::utils::vec2 point = path.ToCartesian(s0, rho0);
          geometry_msgs::PoseStamped pose;
          pose.pose.position.x = point.x;
          pose.pose.position.y = point.y;
          local_path.poses.push_back(pose);
          s0 += output_path_step;
        }
        local_path.header.frame_id = "odom";
        local_path.header.stamp = ros::Time::now();
        local_path.header.seq = loop_count;
        path_pub.publish(local_path);
      }
      else {
        nav_msgs::Path local_path;
        geometry_msgs::PoseStamped pose;
        pose.pose = odom.pose.pose;
        local_path.poses.push_back(pose);
        local_path.header.frame_id = "odom";
        local_path.header.stamp = ros::Time::now();
        local_path.header.seq = loop_count;
        path_pub.publish(local_path);
      }
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
        std::cout << "Local planner did not run because occupancy grid not recieved." << std::endl;
      }
    }
    new_grid_rcvd = false;
    loop_count++;
    double end_secs = ros::WallTime::now().toSec();
    if ((end_secs - start_secs) > 2.5 * dt){
      std::cout << "WARNING: TANG PLANNER TOOK " << (end_secs - start_secs) << " TO COMPLETE. REQUESTED UPDATE SPEED IS " << dt << std::endl;
    }

    ros::spinOnce();
    rosrate.sleep();
  }

  return 0;
}
