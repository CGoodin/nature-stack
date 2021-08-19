#include "avt_341/visualization/x11_visualizer.h"

namespace avt_341 {
  namespace visualization {


    bool X11Visualizer::initialize_display(int nx, int ny) {
      disp_.resize(nx, ny);
      image_ = cimg_library::CImg<float>();
      image_.assign(nx, ny, 1, 3, 0.0f);
      return true;
    }

    void X11Visualizer::draw_point(const int x0, const int y0, const avt_341::utils::vec3 &color) {
      image_.draw_point(x0,y0, (float *)&color);
    }

    void X11Visualizer::draw_line(const int x0, const int y0, const int x1, const int y1,
                                  const avt_341::utils::vec3 &color) {
      image_.draw_line(x0, y0, x1, y1, (float *)&color);
    }

    void X11Visualizer::display() {
      auto img = image_.get_mirror('y');
      disp_ = img;
    }

    void X11Visualizer::save(const std::string &file_name) {
      image_.save(file_name.c_str());
    }

    void X11Visualizer::save(const std::string &file_name, int nx, int ny) {
      auto img = image_.get_resize(nx,ny);
      img.save(file_name.c_str());
    }

    void X11Visualizer::draw_circle(const int x0, const int y0, int radius, const avt_341::utils::vec3 &color) {
      image_.draw_circle(x0, y0, radius, (float *)&color);
    }
  }
}