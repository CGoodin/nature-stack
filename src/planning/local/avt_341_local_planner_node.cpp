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
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"
#include "rosgraph_msgs/Clock.h"
// avt_341 includes
#include "avt_341/planning/local/spline_planner.h"
#include "avt_341/planning/local/spline_plotter.h"

avt_341::msg::Odometry odom;
avt_341::msg::OccupancyGrid grid;
avt_341::msg::Path global_path;
avt_341::msg::Path waypoints;
bool odom_rcvd = false;
bool new_grid_rcvd = false;

void OdometryCallback(avt_341::msg::OdometryPtr &rcv_odom){
  odom = *rcv_odom;
  odom_rcvd = true;
}

void GridCallback(avt_341::msg::OccupancyGridPtr &rcv_grid){
  grid = *rcv_grid;
  new_grid_rcvd = true;
}

void PathCallback(avt_341::msg::PathPtr rcv_path){
  global_path = *rcv_path;
}

void WaypointCallback(avt_341::msg::PathPtr wp_path){
  waypoints = *wp_path;
}

int main(int argc, char *argv[]){

  auto n = avt_341::node::init_node(argc, argv, "avt_341_planner_node");

  // Create publishers and subscribers
  auto path_pub = n->create_publisher<avt_341::msg::Path>("avt_341/local_path", 10);
  auto odometry_sub = n->create_subscription("avt_341/odometry", 10, OdometryCallback);
  auto grid_sub = n->create_subscription("avt_341/occupancy_grid", 10, GridCallback);
  auto path_sub = n->create_subscription("avt_341/global_path", 10, PathCallback);
  auto wp_sub = n->create_subscription("avt_341/waypoints", 10, WaypointCallback);

  avt_341::planning::Planner planner;
  // planner params
    float path_look_ahead, vehicle_width, max_steer_angle, output_path_step, path_int_step, rate;
    int dilation_factor, num_paths;
    float w_c, w_d, w_s, w_r;
    bool trim_path, display;
    std::string display_type;

    n->get_parameter("~path_look_ahead", path_look_ahead, 15.0f);
    n->get_parameter("~vehicle_width", vehicle_width, 3.0f);
    n->get_parameter("~num_paths", num_paths, 31);
    n->get_parameter("~max_steer_angle", max_steer_angle, 0.43f);
    n->get_parameter("~output_path_step", output_path_step, 0.5f);
    n->get_parameter("~path_integration_step", path_int_step, 0.25f);
    n->get_parameter("~dilation_factor", dilation_factor, 0);
    n->get_parameter("~w_c", w_c, 0.2f);
    n->get_parameter("~w_d", w_d, 0.2f);
    n->get_parameter("~w_s", w_s, 0.2f);
    n->get_parameter("~w_r", w_r, 0.4f);
    n->get_parameter("~rate", rate, 50.0f);
    n->get_parameter("~trim_path", trim_path, false);
    n->get_parameter("~display", display, false);

  avt_341::planning::Plotter plotter;

  unsigned int loop_count = 0;
  float dt = 1.0f / rate;
  float elapsed_time = 0.0f;
  avt_341::node::Rate rosrate(rate);
  while (avt_341::node::ok()){
    double start_secs = n->get_now_seconds();
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
        avt_341::msg::Path local_path;
        avt_341::planning::Candidate best = planner.GetBestPath();
        float s0 = best.GetS0() + ds;
        float s_max = s0 + best.GetMaxLength() - ds;
        while (s0 < s_max){
          float rho0 = best.At(s0 - best.GetS0());
          avt_341::utils::vec2 point = path.ToCartesian(s0, rho0);
          avt_341::msg::PoseStamped pose;
          pose.pose.position.x = point.x;
          pose.pose.position.y = point.y;
          local_path.poses.push_back(pose);
          s0 += output_path_step;
        }
        local_path.header.frame_id = "odom";
        local_path.header.stamp = n->get_stamp();
        local_path.header.seq = loop_count;
        path_pub->publish(local_path);
      }
      else {
        avt_341::msg::Path local_path;
        avt_341::msg::PoseStamped pose;
        pose.pose = odom.pose.pose;
        local_path.poses.push_back(pose);
        local_path.header.frame_id = "odom";
        local_path.header.stamp = n->get_stamp();
        local_path.header.seq = loop_count;
        path_pub->publish(local_path);
      }
      odom_rcvd = false;
    }
    else {
      if (global_path.poses.size() <= 0){
        std::cout << "Local planner did not run because global path not received " << std::endl;
      }
      else if (!odom_rcvd){
        std::cout << "Local planner did not run because vehicle odometry not received." << std::endl;
      }
      else if (grid.data.size() <= 0){
        std::cout << "Local planner did not run because occupancy grid not received." << std::endl;
      }
    }
    new_grid_rcvd = false;
    loop_count++;
    double end_secs = n->get_now_seconds();
    if ((end_secs - start_secs) > 2.5 * dt){
      std::cout << "WARNING: TANG PLANNER TOOK " << (end_secs - start_secs) << " TO COMPLETE. REQUESTED UPDATE SPEED IS " << dt << std::endl;
    }

    n->spin_some();
    rosrate.sleep();
  }

  return 0;
}
