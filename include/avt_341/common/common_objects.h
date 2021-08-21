//
// Created by Stefan on 2021-07-28.
//

#ifndef AVT_341_COMMON_OBJECTS_H
#define AVT_341_COMMON_OBJECTS_H

#include "sensor_msgs/msg/point_cloud.hpp"
#include "avt_341/avt_341_utils.h"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace avt_341 {
  namespace common {

    struct vec4_d {
      vec4_d()= default;
      vec4_d(double x, double y, double z, double w): x(x), y(y), z(z), w(w){ }
      double x;
      double y;
      double z;
      double w;
    };

    struct vec3_d {
      vec3_d()= default;
      vec3_d(double x, double y, double z): x(x), y(y), z(z){ }
      double x;
      double y;
      double z;
    };

    struct Header{
      std::string frame_id;
      double seconds;
      int32_t sec;
      int32_t nanosec;
    };

    struct GridOrigin {
      vec3_d position;
      vec4_d orientation;
    };

    struct GridInfo{
      GridOrigin origin;
      float resolution;
      uint32_t width;
      uint32_t height;
    };

    struct OccupancyGrid{
      Header header;
      GridInfo info;
    };


    struct Pose{
      vec3_d position;
      vec4_d orientation;
    };

    struct PoseStamped{
      Pose pose;
      Header header;
    };

    struct Odometry {
      Pose pose;
    };

    struct Path {
      std::vector<PoseStamped> poses;
    };

    struct PointCloud{
      std::vector<utils::vec3> points;
      size_t size() const{
        return points.size();
      }
    };

    struct Twist{
      vec3_d linear;
      vec3_d angular;
    };
  }
}

#endif //AVT_341_COMMON_OBJECTS_H
