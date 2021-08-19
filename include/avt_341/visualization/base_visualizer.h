#ifndef AVT_341_VISUALIZER_BASE_H
#define AVT_341_VISUALIZER_BASE_H

#include <avt_341/avt_341_utils.h>

namespace avt_341{
  namespace visualization{

    /**
    * Base null visualizer with no image display support
    * Used when RVIZ display is active since RVIZ uses inherit ROS messages for visualization and requires no image display
    */
    class VisualizerBase{
    public:
      VisualizerBase() = default;
      virtual bool initialize_display(int nx, int ny){ return false; }
      virtual void draw_point(const int x0, const int y0, const avt_341::utils::vec3 & color){}
      virtual void draw_circle(const int x0, const int y0, int radius, const avt_341::utils::vec3 & color){}
      virtual void draw_line(const int x0, const int y0, const int x1, const int y1, const avt_341::utils::vec3 & color){}
      virtual void display(){}
      virtual void save(const std::string & file_name){}
      virtual void save(const std::string & file_name, int nx, int ny){}
    };

  }
}


#endif //AVT_341_VISUALIZER_BASE_H
