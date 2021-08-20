#include <limits>
#include "avt_341/planning/local/rviz_spline_plotter.h"
#include <avt_341/planning/local/spline_path.h>

namespace avt_341 {
  namespace planning{

    RVIZPlotter::RVIZPlotter(std::shared_ptr<avt_341::visualization::VisualizerBase> visualizer,
        std::shared_ptr<avt_341::node::NodeProxy> node) : Plotter(visualizer), node_(node) {
        candidate_paths_publisher = node->create_publisher<avt_341::msg::Marker>("avt_341/candidate_paths", 1);
        blocked_paths_publisher = node->create_publisher<avt_341::msg::Marker>("avt_341/candidate_paths_blocked", 1);
    }

    avt_341::msg::Marker RVIZPlotter::get_marker_msg(bool is_blocked) const{
      avt_341::msg::Marker marker;
      marker.header.frame_id = "odom";
      marker.header.stamp = node_->get_stamp();
      marker.id = 0;
      marker.type = avt_341::msg::Marker::LINE_LIST;
      marker.action = avt_341::msg::Marker::MODIFY;
      marker.scale.x = 0.15;
      marker.color.a = 1.0;
      if(is_blocked){
        marker.color.r = 1.0;
      }else{
        marker.color.b = 1.0;
      }
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      return marker;
    }

    void RVIZPlotter::Display(bool save, const std::string & ofname, int nx, int ny) {
      if (!map_set_)return;

      avt_341::msg::Marker candidate_paths_marker = get_marker_msg(false);
      avt_341::msg::Marker blocked_paths_marker = get_marker_msg(true);

      float ds = pixdim_;
      Path wp_path(path_);
      for (int i = 0; i < curves_.size(); i++) {
        float s0 = curves_[i].GetS0() + ds;
        float s_max = s0 + curves_[i].GetMaxLength() - ds;
        bool hits_obstacle = curves_[i].HitsObstacle();
        while (s0 < s_max){
          float rho0 = curves_[i].At(s0- curves_[i].GetS0());
          float s1 = s0 + pixdim_;
          float rho1 = curves_[i].At(s1- curves_[i].GetS0());
          avt_341::utils::vec2 pc0 = wp_path.ToCartesian(s0, rho0);
          avt_341::utils::vec2 pc1 = wp_path.ToCartesian(s1, rho1);
          if (std::isnan(pc0.x) || std::isnan(pc0.y) || std::isnan(pc1.x) || std::isnan(pc1.y)){
            break;
          }
          avt_341::msg::Point p0;
          avt_341::msg::Point p1;
          p0.x = pc0.x;
          p0.y = pc0.y;
          p0.z = 0.0;
          p1.x = pc1.x;
          p1.y = pc1.y;
          p1.z = 0.0;
          if(hits_obstacle){
            blocked_paths_marker.points.push_back(p0);
            blocked_paths_marker.points.push_back(p1);
          }else{
            candidate_paths_marker.points.push_back(p0);
            candidate_paths_marker.points.push_back(p1);
          }
          s0 += pixdim_;
        }
      }

      candidate_paths_marker.action = candidate_paths_marker.points.empty() > 0 ? avt_341::msg::Marker::DELETE : avt_341::msg::Marker::MODIFY;
      blocked_paths_marker.action = blocked_paths_marker.points.empty() ? avt_341::msg::Marker::DELETE : avt_341::msg::Marker::MODIFY;
      candidate_paths_publisher->publish(candidate_paths_marker);
      blocked_paths_publisher->publish(blocked_paths_marker);

    }

  } // namespace planning
} // namespace avt_341
