#include <limits>
#include "nature/planning/local/rviz_spline_plotter.h"
#include <nature/planning/local/spline_path.h>
#include <sstream>

namespace nature {
  namespace planning{

    RVIZPlotter::RVIZPlotter(std::shared_ptr<nature::visualization::VisualizerBase> visualizer, const std::string & cost_vis,
                             std::shared_ptr<nature::node::NodeProxy> node, float w_c, float w_s, float w_r, float w_d, float w_t, float cost_vis_text_size_) : Plotter(visualizer), cost_vis_(cost_vis), node_(node),
                                                                                                                                                      w_c_(w_c), w_s_(w_s), w_r_(w_r), w_d_(w_d), w_t_(w_t), cost_vis_text_size_(cost_vis_text_size_) {
      candidate_paths_publisher = node->create_publisher<nature::msg::MarkerArray>("nature/candidate_paths", 1);
    }

    nature::msg::Marker RVIZPlotter::get_marker_msg(int type, int id, bool is_blocked) const{
      nature::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = node_->get_stamp();
      marker.id = id;
      marker.type = type;
      marker.action = nature::msg::Marker::MODIFY;
      marker.color.a = 1.0;
      if(type == nature::msg::Marker::LINE_LIST){
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

      nature::msg::Marker candidate_paths_marker = get_marker_msg(nature::msg::Marker::LINE_LIST, 0, false);
      nature::msg::Marker blocked_paths_marker = get_marker_msg(nature::msg::Marker::LINE_LIST, 1, true);

      float ds = pixdim_;
      Path wp_path(path_);
      std::vector<nature::utils::vec2> paths_last_points;

      for (int i = 0; i < curves_.size(); i++) {
        float s0 = curves_[i].GetS0() + ds;
        float s_max = s0 + curves_[i].GetMaxLength() - ds;
        bool hits_obstacle = curves_[i].HitsObstacle();
        nature::utils::vec2 pc1;

        while (s0 < s_max){
          float rho0 = curves_[i].At(s0- curves_[i].GetS0());
          float s1 = s0 + pixdim_;
          float rho1 = curves_[i].At(s1- curves_[i].GetS0());
          nature::utils::vec2 pc0 = wp_path.ToCartesian(s0, rho0);
          pc1 = wp_path.ToCartesian(s1, rho1);
          if (std::isnan(pc0.x) || std::isnan(pc0.y) || std::isnan(pc1.x) || std::isnan(pc1.y)){
            break;
          }
          nature::msg::Point p0;
          nature::msg::Point p1;
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

      candidate_paths_marker.action = candidate_paths_marker.points.empty() > 0 ? nature::msg::Marker::DELETE : nature::msg::Marker::MODIFY;
      blocked_paths_marker.action = blocked_paths_marker.points.empty() ? nature::msg::Marker::DELETE : nature::msg::Marker::MODIFY;
      nature::msg::MarkerArray marker_array;
      marker_array.markers.push_back(candidate_paths_marker);
      marker_array.markers.push_back(blocked_paths_marker);

      if(cost_vis_ != "none"){
        for(int j = 0; j < curves_.size(); j++){
          nature::msg::Marker text_marker = get_marker_msg(nature::msg::Marker::TEXT_VIEW_FACING, marker_array.markers.size()+1);

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
            float t = w_t_ * curves_[j].GetSegmentationCost();
            if(s > 1e-3) out << " s: " << s;
            if(c > 1e-3) out << " c: " << c;
            if(r > 1e-3) out << " r: " << r;
            if(d > 1e-3) out << " d: " << d;
            if(t > 1e-3) out << " t: " << t;
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
} // namespace nature