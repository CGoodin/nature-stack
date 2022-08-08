#include "nature/visualization/image_visualizer.h"

namespace nature {
  namespace visualization {


    bool ImageVisualizer::initialize_display(int nx, int ny) {
      disp_.resize(nx, ny);
      image_ = cimg_library::CImg<float>();
      image_.assign(nx, ny, 1, 3, 0.0f);
      return true;
    }

    void ImageVisualizer::draw_point(const int x0, const int y0, const nature::utils::vec3 &color) {
      image_.draw_point(x0,y0, (float *)&color);
    }

    void ImageVisualizer::draw_line(const int x0, const int y0, const int x1, const int y1,
                                    const nature::utils::vec3 &color) {
      image_.draw_line(x0, y0, x1, y1, (float *)&color);
    }

    void ImageVisualizer::display() {
      auto img = image_.get_mirror('y');
      disp_ = img;
    }

    void ImageVisualizer::save(const std::string &file_name) {
      image_.save(file_name.c_str());
    }

    void ImageVisualizer::save(const std::string &file_name, int nx, int ny) {
      auto img = image_.get_resize(nx,ny);
      img.save(file_name.c_str());
    }

    void ImageVisualizer::draw_circle(const int x0, const int y0, int radius, const nature::utils::vec3 &color) {
      image_.draw_circle(x0, y0, radius, (float *)&color);
    }
  }
}