//
// Created by stefan on 2021-08-19.
//

#ifndef NATURE_RVIZ_SPLINE_PLOTTER_H
#define NATURE_RVIZ_SPLINE_PLOTTER_H

#include "nature/planning/local/spline_plotter.h"
#include "nature/visualization/base_visualizer.h"
#include "nature/node/ros_types.h"
#include "nature/node/node_proxy.h"

namespace nature {
  namespace planning{

    class RVIZPlotter : public Plotter {
    public:
      RVIZPlotter(std::shared_ptr<nature::visualization::VisualizerBase> visualizer, const std::string & cos_vis,
                  std::shared_ptr<nature::node::NodeProxy> node, float w_c, float w_s, float w_r, float w_d, float w_t, float cost_vis_text_size_);
      virtual void Display(bool save, const std::string & ofname, int nx, int ny) override;

    private:
      nature::msg::Marker get_marker_msg(int type, int id, bool is_blocked = false) const;
      std::shared_ptr<nature::node::NodeProxy> node_;
      std::string cost_vis_;
      float cost_vis_text_size_;
      std::shared_ptr<nature::node::Publisher<nature::msg::MarkerArray>> candidate_paths_publisher;
      float w_c_;
      float w_d_;
      float w_r_;
      float w_s_;
      float w_t_;
    };
  } // namespace planning
} // namespace nature

#endif //NATURE_RVIZ_SPLINE_PLOTTER_H