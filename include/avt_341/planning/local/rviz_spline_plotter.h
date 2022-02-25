//
// Created by stefan on 2021-08-19.
//

#ifndef AVT_341_RVIZ_SPLINE_PLOTTER_H
#define AVT_341_RVIZ_SPLINE_PLOTTER_H

#include "avt_341/planning/local/spline_plotter.h"
#include "avt_341/visualization/base_visualizer.h"
#include "avt_341/node/ros_types.h"
#include "avt_341/node/node_proxy.h"

namespace avt_341 {
  namespace planning{

    class RVIZPlotter : public Plotter {
    public:
      RVIZPlotter(std::shared_ptr<avt_341::visualization::VisualizerBase> visualizer, const std::string & cos_vis,
                  std::shared_ptr<avt_341::node::NodeProxy> node, float w_c, float w_s, float w_r, float w_d, float w_t, float cost_vis_text_size_);
      virtual void Display(bool save, const std::string & ofname, int nx, int ny) override;

    private:
      avt_341::msg::Marker get_marker_msg(int type, int id, bool is_blocked = false) const;
      std::shared_ptr<avt_341::node::NodeProxy> node_;
      std::string cost_vis_;
      float cost_vis_text_size_;
      std::shared_ptr<avt_341::node::Publisher<avt_341::msg::MarkerArray>> candidate_paths_publisher;
      float w_c_;
      float w_d_;
      float w_r_;
      float w_s_;
      float w_t_;
    };
  } // namespace planning
} // namespace avt_341

#endif //AVT_341_RVIZ_SPLINE_PLOTTER_H