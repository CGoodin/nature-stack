#ifndef NATURE_POINT_CLOUD_GENERATOR_H
#define NATURE_POINT_CLOUD_GENERATOR_H

#include "nature/nature_utils.h"

namespace  nature {
  namespace perception {
    class PointCloudGenerator {
      public:
        static void toROSMsg(const std::vector<nature::utils::vec3> & points, nature::msg::PointCloud2 & out_point_cloud);
        static void toROSMsg(const std::vector<nature::utils::vec3> & points, const std::vector<int> & seg_values, nature::msg::PointCloud2 & out_point_cloud);
    };
  }
}

#endif //NATURE_POINT_CLOUD_GENERATOR_H
