#ifndef AVT_341_POINT_CLOUD_GENERATOR_H
#define AVT_341_POINT_CLOUD_GENERATOR_H

#include "avt_341/avt_341_utils.h"

namespace  avt_341 {
  namespace perception {
    class PointCloudGenerator {
      public:
        static void toROSMsg(const std::vector<avt_341::utils::vec3> & points, avt_341::msg::PointCloud2 & out_point_cloud);
    };
  }
}

#endif //AVT_341_POINT_CLOUD_GENERATOR_H
