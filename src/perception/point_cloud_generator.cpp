#include "avt_341/perception/point_cloud_generator.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


void avt_341::perception::PointCloudGenerator::toROSMsg(const std::vector<avt_341::utils::vec3> & points, avt_341::msg::PointCloud2 & out_point_cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  for(const auto & p : points){
    point_cloud.push_back(pcl::PointXYZ(p.x, p.y, p.z));
  }
  pcl::toROSMsg(point_cloud, out_point_cloud);
}
