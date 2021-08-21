#ifndef AVT_341_IMAGE_VISUALIZER_H
#define AVT_341_IMAGE_VISUALIZER_H

#include <opencv2/core/mat.hpp>
#include "opencv2/highgui.hpp"
#include "avt_341/visualization/base_visualizer.h"

namespace avt_341{
  namespace visualization {

    class ImageVisualizer  : public VisualizerBase {

    public:
      bool initialize_display(int nx, int ny) override;
      void draw_point(const int x0, const int y0, const avt_341::utils::vec3 &color) override;
      void draw_line(const int x0, const int y0, const int x1, const int y1, const avt_341::utils::vec3 &color) override;
      void draw_circle(const int x0, const int y0, int radius, const avt_341::utils::vec3 &color) override;
      void display() override;
      void save(const std::string &file_name);
      void save(const std::string &file_name, int nx, int ny) override;

    private:
      cv::Mat image_;
    };

  }
}
#endif //AVT_341_IMAGE_VISUALIZER_H
