#include "nature/perception/point_cloud_generator.h"


#ifdef ROS_1

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// union used to convert floats to bytes
union dataUnion
{
    float f;
    uint8_t b[sizeof(float)];
};

void nature::perception::PointCloudGenerator::toROSMsg(const std::vector<nature::utils::vec3> & points, nature::msg::PointCloud2 & out_point_cloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  for(const auto & p : points){
    point_cloud.push_back(pcl::PointXYZ(p.x, p.y, p.z));
  }
  pcl::toROSMsg(point_cloud, out_point_cloud);
}

void nature::perception::PointCloudGenerator::toROSMsg(const std::vector<nature::utils::vec3> & points, const std::vector<int> & seg_values, nature::msg::PointCloud2 & out_point_cloud) {
    int num_lidar_points = points.size();
    out_point_cloud.height = 1;
    out_point_cloud.width = num_lidar_points;
    std::vector<nature::msg::PointField> fields(4);
    fields[0].name = "x";
    fields[1].name = "y";
    fields[2].name = "z";
    fields[3].name = "segmentation";
    uint32_t offset = 0ul;
    for (int i = 0; i < fields.size(); i++)
    {
        fields[i].offset = offset;
        fields[i].datatype = nature::msg::PointField::FLOAT32;
        fields[i].count = 1;
        offset += 4;
    }
    out_point_cloud.fields = fields;
    out_point_cloud.is_bigendian = false;
    out_point_cloud.point_step = offset;
    out_point_cloud.row_step = out_point_cloud.point_step * out_point_cloud.width;
    out_point_cloud.is_dense = true;
    std::vector<uint8_t> data;
    data.reserve(4 * num_lidar_points * sizeof(float));
    for (int v_i = 0; v_i < points.size(); v_i++)
    {
        nature::utils::vec3 v = points[v_i];
        float seg_value = (float)seg_values[v_i];
        for(int i = 0; i < 3; i++){
            dataUnion u;
            u.f = v[i];
            for (int j = 0; j < sizeof(float); j++)
                data.push_back(u.b[j]);
        }
        dataUnion u;
        u.f = seg_value;
        for (int j = 0; j < sizeof(float); j++)
            data.push_back(u.b[j]);
    }
    out_point_cloud.data = data;
}

#else


// union used to convert floats to bytes
union dataUnion
{
  float f;
  uint8_t b[sizeof(float)];
};

void nature::perception::PointCloudGenerator::toROSMsg(const std::vector<nature::utils::vec3> & points, nature::msg::PointCloud2 & out_point_cloud) {

  int num_lidar_points = points.size();
  out_point_cloud.height = 1;
  out_point_cloud.width = num_lidar_points;
  std::vector<nature::msg::PointField> fields(3);
  fields[0].name = "x";
  fields[1].name = "y";
  fields[2].name = "z";
  uint32_t offset = 0ul;
  for (int i = 0; i < fields.size(); i++)
  {
    fields[i].offset = offset;
    fields[i].datatype = nature::msg::PointField::FLOAT32;
    fields[i].count = 1;
    offset += 4;
  }
  out_point_cloud.fields = fields;
  out_point_cloud.is_bigendian = false;
  out_point_cloud.point_step = offset;
  out_point_cloud.row_step = out_point_cloud.point_step * out_point_cloud.width;
  out_point_cloud.is_dense = true;
  std::vector<uint8_t> data;
  data.reserve(3 * num_lidar_points * sizeof(float));
  for (const nature::utils::vec3 & v : points)
  {
    for(int i = 0; i < 3; i++){
      dataUnion u;
      u.f = v[i];
      for (int j = 0; j < sizeof(float); j++)
      {
        data.push_back(u.b[j]);
      }
    }
  }
  out_point_cloud.data = data;
}

void nature::perception::PointCloudGenerator::toROSMsg(const std::vector<nature::utils::vec3> & points, const std::vector<int> & seg_values, nature::msg::PointCloud2 & out_point_cloud) {
    int num_lidar_points = points.size();
    out_point_cloud.height = 1;
    out_point_cloud.width = num_lidar_points;
    std::vector<nature::msg::PointField> fields(4);
    fields[0].name = "x";
    fields[1].name = "y";
    fields[2].name = "z";
    fields[3].name = "segmentation";
    uint32_t offset = 0ul;
    for (int i = 0; i < fields.size(); i++)
    {
        fields[i].offset = offset;
        fields[i].datatype = nature::msg::PointField::FLOAT32;
        fields[i].count = 1;
        offset += 4;
    }
    out_point_cloud.fields = fields;
    out_point_cloud.is_bigendian = false;
    out_point_cloud.point_step = offset;
    out_point_cloud.row_step = out_point_cloud.point_step * out_point_cloud.width;
    out_point_cloud.is_dense = true;
    std::vector<uint8_t> data;
    data.reserve(4 * num_lidar_points * sizeof(float));
    for (int v_i = 0; v_i < points.size(); v_i++)
    {
        nature::utils::vec3 v = points[v_i];
        float seg_value = (float)seg_values[v_i];
        for(int i = 0; i < 3; i++){
            dataUnion u;
            u.f = v[i];
            for (int j = 0; j < sizeof(float); j++)
                data.push_back(u.b[j]);
        }
        dataUnion u;
        u.f = seg_value;
        for (int j = 0; j < sizeof(float); j++)
            data.push_back(u.b[j]);
    }
    out_point_cloud.data = data;
}

#endif // ROS_1