/**
 * \file nature_planner_node.cpp
 * Plan a local trajectory using a global path.
 * 
 * \author Chris Goodin
 *
 * \contact cgoodin@cavs.msstate.edu
 * 
 * \date 8/31/2020
 */
// ROS includes
#include "nature/node/ros_types.h"
#include "nature/node/node_proxy.h"
// nature includes
#include "nature/planning/local/spline_planner.h"
#include "nature/planning/local/spline_plotter.h"
#include "nature/visualization/visualization_factory.h"

nature::msg::Odometry odom;
nature::msg::OccupancyGrid grid;
nature::msg::OccupancyGrid segmentation_grid;
nature::msg::Path global_path;
nature::msg::Path waypoints;
bool odom_rcvd = false;
bool new_grid_rcvd = false;
bool new_seg_grid_rcvd = false;

void OdometryCallback(nature::msg::OdometryPtr rcv_odom){
  odom = *rcv_odom;
  odom_rcvd = true;
}

void GridCallback(nature::msg::OccupancyGridPtr rcv_grid){
  grid = *rcv_grid;
  new_grid_rcvd = true;
}

void SegmentationGridCallback(nature::msg::OccupancyGridPtr rcv_grid){
    segmentation_grid = *rcv_grid;
    new_seg_grid_rcvd = true;
}

void PathCallback(nature::msg::PathPtr rcv_path){
  global_path = *rcv_path;
}

void WaypointCallback(nature::msg::PathPtr wp_path){
  waypoints = *wp_path;
}

int main(int argc, char *argv[]){

  auto n = nature::node::init_node(argc, argv, "nature_planner_node");

  // Create publishers and subscribers
  auto path_pub = n->create_publisher<nature::msg::Path>("nature/local_path", 10);
  auto odometry_sub = n->create_subscription<nature::msg::Odometry>("nature/odometry", 10, OdometryCallback);
  auto grid_sub = n->create_subscription<nature::msg::OccupancyGrid>("nature/occupancy_grid", 10, GridCallback);
  auto segmentation_grid_sub = n->create_subscription<nature::msg::OccupancyGrid>("nature/segmentation_grid", 10, SegmentationGridCallback);
  auto path_sub = n->create_subscription<nature::msg::Path>("nature/global_path", 10, PathCallback);
  auto wp_sub = n->create_subscription<nature::msg::Path>("nature/waypoints", 10, WaypointCallback);

  nature::planning::Planner planner;
  // planner params
  float path_look_ahead, vehicle_width, max_steer_angle, output_path_step, path_int_step, rate;
  int dilation_factor, num_paths;
  float w_c, w_d, w_s, w_r, w_t, cost_vis_text_size, ignore_coll_before_dist;
  bool trim_path, use_global_path, use_blend;
  std::string display, cost_vis;
  bool keep_good_path;
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
  n->get_parameter("~w_t", w_t, 0.0f);
  n->get_parameter("~rate", rate, 50.0f);
  n->get_parameter("~ignore_coll_before_dist", ignore_coll_before_dist, 0.0f);
  n->get_parameter("~trim_path", trim_path, false);
  n->get_parameter("~use_global_path", use_global_path, false);
  n->get_parameter("~keep_good_path", keep_good_path, false);
  n->get_parameter("~use_blend", use_blend, true);
  n->get_parameter("~cost_vis", cost_vis, std::string("final"));
  n->get_parameter("~cost_vis_text_size", cost_vis_text_size, 2.0f);
  n->get_parameter("~display", display, nature::visualization::default_display);

  planner.SetArcLengthIntegrationStep(path_int_step);
  planner.SetComfortabilityWeight(w_c);
  planner.SetDynamicSafetyWeight(w_d);
  planner.SetStaticSafetyWeight(w_s);
  planner.SetPathAdherenceWeight(w_r);
  planner.SetSegmentationFactorWeight(w_t);
  planner.SetUseBlend(use_blend);
  planner.SetIgnoreCollBeforeDist(ignore_coll_before_dist);

  std::shared_ptr<nature::planning::Plotter> plotter = nature::visualization::create_local_path_plotter(display, cost_vis, n,
                                                                                                          planner.GetComfortabilityWeight(), planner.GetStaticSafetyWeight(),
                                                                                                          planner.GetPathAdherenceWeight(), planner.GetDynamicSafetyWeight(),
                                                                                                          planner.GetSegmentationWeight(), cost_vis_text_size);

  unsigned int loop_count = 0;
  float dt = 1.0f / rate;
  float elapsed_time = 0.0f;
  nature::node::Rate rosrate(rate);
  float path_age = 0.0f;
  float s_old = 0.0f;
  bool old_path_still_good = false;
  while (nature::node::ok()){
    double start_secs = n->get_now_seconds();
    if (global_path.poses.size() > 0 && odom_rcvd && grid.data.size() > 0){

      std::vector<nature::utils::vec2> path_points;
      if (use_global_path){
        for (int i = 0; i < global_path.poses.size(); i++){
          nature::utils::vec2 point(global_path.poses[i].pose.position.x, global_path.poses[i].pose.position.y);
          path_points.push_back(point);
        }
      }
      else{
        for (int i = 0; i < waypoints.poses.size(); i++){
          nature::utils::vec2 point(waypoints.poses[i].pose.position.x, waypoints.poses[i].pose.position.y);
          path_points.push_back(point);
        }
      }

      nature::planning::Path path;
      if (trim_path && use_global_path ){
        nature::utils::vec2 current_pos(odom.pose.pose.position.x, odom.pose.pose.position.y);
        path.Init(path_points, current_pos, 1.5f * path_look_ahead);
      }
      else{
        path.Init(path_points);
      }
      

      path.FixBeginning(odom.pose.pose.position.x, odom.pose.pose.position.y);
      path.FixEnd();

      std::vector<nature::utils::vec2> culled_points = path.GetPoints();
      float s_max = path.GetTotalLength();
      nature::utils::vec2 srho = path.ToSRho(odom.pose.pose.position.x, odom.pose.pose.position.y);
      float s = srho.x;
      float rho_start = srho.y;
      nature::utils::vec2 pconv = path.ToCartesian(s,rho_start);
      float s_lookahead = std::min(path_look_ahead, s_max - s);
      float theta = nature::utils::GetHeadingFromOrientation(odom.pose.pose.orientation);
      nature::planning::CurveInfo ci = path.GetCurvatureAndAngle(s);
      path_age += dt;

      float ds = s-s_old;
      if (path_age>1.0f || ds>0.5f*path_look_ahead || !old_path_still_good || !keep_good_path){
        planner.GeneratePaths(num_paths, s, rho_start, theta - ci.theta, s_lookahead, max_steer_angle, vehicle_width);
        planner.SetCenterline(path);
        path_age = 0.0f;
        s_old = s;
      }
  
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
      if (new_seg_grid_rcvd) planner.DilateGrid(segmentation_grid, dilation_factor, llx, lly, urx, ury);
      // Note: if grid size gets large, DilateGrid can take a significant amount of time

      // most of the calculation time spent on this function call
      bool path_found = planner.CalculateCandidateCosts(grid, segmentation_grid, odom);
      if (!path_found){
        old_path_still_good = false;
      }
      else{
        old_path_still_good = true;
      }

      if (display != "none"){
        plotter->AddMap(grid);
        plotter->SetPath(culled_points);
        plotter->AddWaypoints(waypoints);
        std::vector<nature::planning::Candidate> paths = planner.GetCandidates();
        plotter->AddCurves(paths);
        plotter->Display();
      }

      if (path_found){
        float ds = output_path_step;
        nature::msg::Path local_path;
        nature::planning::Candidate best = planner.GetBestPath();
        float s0 = best.GetS0() + ds;
        float s_max = s0 + best.GetMaxLength() - ds;
        while (s0 < s_max){
          float rho0 = best.At(s0 - best.GetS0());
          nature::utils::vec2 point = path.ToCartesian(s0, rho0);
          nature::msg::PoseStamped pose;
          pose.pose.position.x = point.x;
          pose.pose.position.y = point.y;
          local_path.poses.push_back(pose);
          s0 += output_path_step;
        }
        //local_path.header.frame_id = "odom";
        local_path.header.frame_id = "map";
        local_path.header.stamp = n->get_stamp();
        nature::node::set_seq(local_path.header, loop_count);
        path_pub->publish(local_path);
      }
      else {
        nature::msg::Path local_path;
        nature::msg::PoseStamped pose;
        pose.pose = odom.pose.pose;
        local_path.poses.push_back(pose);
        //local_path.header.frame_id = "odom";
        local_path.header.frame_id = "map";
        local_path.header.stamp = n->get_stamp();
        nature::node::set_seq(local_path.header, loop_count);
        path_pub->publish(local_path);
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
    elapsed_time += dt;
    n->spin_some();
    rosrate.sleep();
  }

  return 0;
}
