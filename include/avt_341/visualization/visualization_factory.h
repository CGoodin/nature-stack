#ifndef AVT_341_VISUALIZATION_FACTORY_H
#define AVT_341_VISUALIZATION_FACTORY_H

#include "avt_341/visualization/base_visualizer.h"
#include "avt_341/visualization/image_visualizer.h"
#include "avt_341/planning/local/rviz_spline_plotter.h"

namespace avt_341{
  namespace visualization{

    const std::string default_display = "x11";    // x11, opencv, rviz, none

    inline std::shared_ptr<avt_341::visualization::VisualizerBase> create_visualizer(const std::string & display_type){
      return display_type == "image" ? std::make_shared<avt_341::visualization::ImageVisualizer>()
                                      : std::make_shared<avt_341::visualization::VisualizerBase>();
    }

    inline std::shared_ptr<avt_341::planning::Plotter> create_local_path_plotter(const std::string & display_type, std::shared_ptr<avt_341::node::NodeProxy> node){
      auto visualizer = create_visualizer(display_type);
      return display_type == "rviz" ? std::make_shared<avt_341::planning::RVIZPlotter>(visualizer, node)
          : std::make_shared<avt_341::planning::Plotter>(visualizer);
    }

  }
}


#endif //AVT_341_VISUALIZATION_FACTORY_H
