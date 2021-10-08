//
// Created by Stefan on 2021-07-28.
//

#ifndef AVT_341_ROS_TYPES_H
#define AVT_341_ROS_TYPES_H

#include <geometry_msgs/Quaternion.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "tf/LinearMath/Transform.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"

namespace avt_341 {
    namespace msg {
        using PointCloud = sensor_msgs::PointCloud;
        using PointCloudPtr = const sensor_msgs::PointCloud::ConstPtr &;

        using PointCloud2 = sensor_msgs::PointCloud2;
        using PointCloud2Ptr = const sensor_msgs::PointCloud2::ConstPtr &;

        using PointField = sensor_msgs::PointField;
        using PointFieldPtr = const sensor_msgs::PointField::ConstPtr &;

        using JointState = sensor_msgs::JointState;
        using JointStatePtr = const sensor_msgs::JointState::ConstPtr &;

        using Twist = geometry_msgs::Twist;
        using TwistPtr = const geometry_msgs::Twist::ConstPtr &;

        using Point32 = geometry_msgs::Point32;
        using Point32Ptr = const geometry_msgs::Point32::ConstPtr &;

        using Quaternion = geometry_msgs::Quaternion;
        using QuaternionPtr = const geometry_msgs::Quaternion::ConstPtr &;

        using Point = geometry_msgs::Point;
        using PointPtr = const geometry_msgs::Point::ConstPtr &;

        using PoseStamped = geometry_msgs::PoseStamped;
        using PoseStampedPtr = const geometry_msgs::PoseStamped::ConstPtr &;

        using PointStamped = geometry_msgs::PointStamped;
        using PointStampedPtr = const geometry_msgs::PointStamped::ConstPtr &;

        using OccupancyGrid = nav_msgs::OccupancyGrid;
        using OccupancyGridPtr = const nav_msgs::OccupancyGrid::ConstPtr &;

        using Path = nav_msgs::Path;
        using PathPtr = const nav_msgs::Path::ConstPtr &;

        using Odometry = nav_msgs::Odometry;
        using OdometryPtr = const nav_msgs::Odometry::ConstPtr &;

        using Marker = visualization_msgs::Marker;
        using MarkerPtr = const visualization_msgs::Marker::ConstPtr &;

        using MarkerArray = visualization_msgs::MarkerArray;
        using MarkerArrayPtr = const visualization_msgs::MarkerArray::ConstPtr &;

        using Float64 = std_msgs::Float64;
        using Float64Ptr = const std_msgs::Float64::ConstPtr &;

        using Int32 = std_msgs::Int32;
        using Int32Ptr = const std_msgs::Int32::ConstPtr &;
    }
    namespace msg_tf{
        using Matrix3x3 = tf::Matrix3x3;
        using Quaternion = tf::Quaternion;
        using Vector3 = tf::Vector3;
    }
}


#endif //AVT_341_ROS_TYPES_H
