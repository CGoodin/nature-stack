#ifndef AVT_341_X11_VISUALIZER_H
#define AVT_341_X11_VISUALIZER_H

#include "avt_341/visualization/base_visualizer.h"
#include "avt_341/CImg.h"

namespace avt_341 {
  namespace visualization {

    class X11Visualizer : public VisualizerBase {

    public:
      X11Visualizer() = default;
      bool initialize_display(int nx, int ny) override;
      void draw_circle(const int x0, const int y0, int radius, const avt_341::utils::vec3 &color) override;
      void draw_point(const int x0, const int y0, const avt_341::utils::vec3 &color) override;
      void draw_line(const int x0, const int y0, const int x1, const int y1, const avt_341::utils::vec3 &color) override;
      void display() override;
      void save(const std::string &file_name) override;
      void save(const std::string &file_name, int nx, int ny) override;

    private:
      cimg_library::CImgDisplay disp_;
      cimg_library::CImg<float> image_;
    };

  }
}
#endif //AVT_341_X11_VISUALIZER_H
