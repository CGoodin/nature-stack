#include <limits>
#include "avt_341/planning/local/rviz_spline_plotter.h"
#include <avt_341/planning/local/spline_path.h>
#include <sstream>

namespace avt_341 {
  namespace planning{

    RVIZPlotter::RVIZPlotter(std::shared_ptr<avt_341::visualization::VisualizerBase> visualizer, const std::string & cost_vis,
                             std::shared_ptr<avt_341::node::NodeProxy> node, float w_c, float w_s, float w_r, float w_d, float cost_vis_text_size_) : Plotter(visualizer), cost_vis_(cost_vis), node_(node),
                             w_c_(w_c), w_s_(w_s), w_r_(w_r), w_d_(w_d), cost_vis_text_size_(cost_vis_text_size_) {
      candidate_paths_publisher = node->create_publisher<avt_341::msg::MarkerArray>("avt_341/candidate_paths", 1);
    }

    avt_341::msg::Marker RVIZPlotter::get_marker_msg(int type, int id, bool is_blocked) const{
      avt_341::msg::Marker marker;
      marker.header.frame_id = "odom";
      marker.header.stamp = node_->get_stamp();
      marker.id = id;
      marker.type = type;
      marker.action = avt_341::msg::Marker::MODIFY;
      marker.color.a = 1.0;
      if(type == avt_341::msg::Marker::LINE_LIST){
        marker.scale.x = 0.15;
      }else{
        marker.scale.z = cost_vis_text_size_;
      }
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

      avt_341::msg::Marker candidate_paths_marker = get_marker_msg(avt_341::msg::Marker::LINE_LIST, 0, false);
      avt_341::msg::Marker blocked_paths_marker = get_marker_msg(avt_341::msg::Marker::LINE_LIST, 1, true);

      float ds = pixdim_;
      Path wp_path(path_);
      std::vector<avt_341::utils::vec2> paths_last_points;

      for (int i = 0; i < curves_.size(); i++) {
        float s0 = curves_[i].GetS0() + ds;
        float s_max = s0 + curves_[i].GetMaxLength() - ds;
        bool hits_obstacle = curves_[i].HitsObstacle();
        avt_341::utils::vec2 pc1;

        while (s0 < s_max){
          float rho0 = curves_[i].At(s0- curves_[i].GetS0());
          float s1 = s0 + pixdim_;
          float rho1 = curves_[i].At(s1- curves_[i].GetS0());
          avt_341::utils::vec2 pc0 = wp_path.ToCartesian(s0, rho0);
          pc1 = wp_path.ToCartesian(s1, rho1);
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
        paths_last_points.push_back(pc1);
      }

      candidate_paths_marker.action = candidate_paths_marker.points.empty() > 0 ? avt_341::msg::Marker::DELETE : avt_341::msg::Marker::MODIFY;
      blocked_paths_marker.action = blocked_paths_marker.points.empty() ? avt_341::msg::Marker::DELETE : avt_341::msg::Marker::MODIFY;
      avt_341::msg::MarkerArray marker_array;
      marker_array.markers.push_back(candidate_paths_marker);
      marker_array.markers.push_back(blocked_paths_marker);

      if(cost_vis_ != "none"){
        for(int j = 0; j < curves_.size(); j++){
          avt_341::msg::Marker text_marker = get_marker_msg(avt_341::msg::Marker::TEXT_VIEW_FACING, marker_array.markers.size()+1);

          std::ostringstream out;
          out.precision(2);
          out << std::fixed;

          if(cost_vis_ == "final" || cost_vis_ == "all"){
            out << curves_[j].GetCost();
          }

          if (cost_vis_ == "components" || cost_vis_ == "all"){
            float s = w_s_ * curves_[j].GetStaticSafety();
            float c = w_c_ * curves_[j].GetComfortability();
            float r = w_r_ * curves_[j].GetRhoCost();
            float d = w_d_ * curves_[j].GetDynamicSafety();
            if(s > 1e-3) out << " s: " << s;
            if(c > 1e-3) out << " c: " << c;
            if(r > 1e-3) out << " r: " << r;
            if(d > 1e-3) out << " d: " << d;
          }
          text_marker.text = out.str();

          text_marker.pose.position.x = paths_last_points[j].x;
          text_marker.pose.position.y = paths_last_points[j].y;
          text_marker.pose.position.z = 0.1;
          marker_array.markers.push_back(text_marker);

        }
      }

      candidate_paths_publisher->publish(marker_array);

    }

  } // namespace planning
} // namespace avt_341