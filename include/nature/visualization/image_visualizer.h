#ifndef NATURE_IMAGE_VISUALIZER_H
#define NATURE_IMAGE_VISUALIZER_H

#include "nature/visualization/base_visualizer.h"
#include "nature/CImg.h"

namespace nature {
  namespace visualization {

    class ImageVisualizer : public VisualizerBase {

    public:
      ImageVisualizer() = default;
      bool initialize_display(int nx, int ny) override;
      void draw_circle(const int x0, const int y0, int radius, const nature::utils::vec3 &color) override;
      void draw_point(const int x0, const int y0, const nature::utils::vec3 &color) override;
      void draw_line(const int x0, const int y0, const int x1, const int y1, const nature::utils::vec3 &color) override;
      void display() override;
      void save(const std::string &file_name) override;
      void save(const std::string &file_name, int nx, int ny) override;

    private:
      cimg_library::CImgDisplay disp_;
      cimg_library::CImg<float> image_;
    };

  }
}
#endif //NATURE_IMAGE_VISUALIZER_H
