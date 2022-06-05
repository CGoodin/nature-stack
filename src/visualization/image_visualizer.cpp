#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "avt_341/visualization/image_visualizer.h"

namespace avt_341 {
  namespace visualization {

    bool ImageVisualizer::initialize_display(int nx, int ny) {
      image_ = cv::Mat(nx, ny, CV_8UC3, cv::Scalar(0, 0, 0));
      return true;
    }

    void ImageVisualizer::draw_point(const int x0, const int y0, const avt_341::utils::vec3 &color) {
      cv::circle(image_, cv::Point(x0, y0), 1, cv::Scalar(color.z, color.y, color.x));
    }

    void ImageVisualizer::draw_line(const int x0, const int y0, const int x1, const int y1,
                                    const avt_341::utils::vec3 &color) {
      cv::line(image_, cv::Point(x0, y0), cv::Point(x1, y1), cv::Scalar(color.z, color.y, color.x));
    }

    void ImageVisualizer::display() {
      cv::Mat image_flip;
      cv::flip(image_, image_flip, 0);
      cv::imshow("Image Display",image_flip);
      cv::waitKey(1);
    }

    void ImageVisualizer::save(const std::string &file_name) {
      cv::imwrite(file_name.c_str(), image_);
    }

    void ImageVisualizer::save(const std::string &file_name, int nx, int ny) {
      auto image_resized = image_.reshape(nx, ny);
      cv::imwrite(file_name.c_str(), image_resized);
    }

    void ImageVisualizer::draw_circle(const int x0, const int y0, int radius, const avt_341::utils::vec3 &color) {
      cv::circle(image_, cv::Point(x0, y0), radius, cv::Scalar(color.z, color.y, color.x));
    }

  }
}