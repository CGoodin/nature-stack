//
// Created by Stefan on 2021-07-28.
//

#ifndef NATURE_CONVERSION_H
#define NATURE_CONVERSION_H

#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.hpp"
#include "geometry_msgs/Pose.hpp"
#include "geometry_msgs/Twist.hpp"
#include "std_msgs/Header.hpp"
#include "nav_msgs/Path.hpp"
#include "nav_msgs/OccupancyGrid.hpp"
#include "nav_msgs/Odometry.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nature/common/common_objects.h"
#include "nature/nature_utils.h"

namespace nature {
    namespace common {

        // Point Cloud
        // ===========================================================================================
        inline common::PointCloud FromRosMsg(const sensor_msgs::msg::PointCloud & point_cloud){
            common::PointCloud pc;
            for(const auto & p: point_cloud.points){
                pc.points.emplace_back(utils::vec3{p.x, p.y, p.z});
            }
            return pc;
        }
        inline sensor_msgs::msg::PointCloud ToRosMsg(const common::PointCloud & point_cloud){
            sensor_msgs::msg::PointCloud pc;
            for(const auto & p: point_cloud.points){
                geometry_msgs::msg::Point32 p_insert;
                p_insert.x = p.x;
                p_insert.y = p.y;
                p_insert.z = p.z;
                pc.points.push_back(p_insert);
            }
            return pc;
        }

        // Twist
        // ===========================================================================================
        inline common::Twist FromRosMsg(const geometry_msgs::msg::Twist & twist){
            common::Twist converted;
            converted.linear.x = twist.linear.x;
            converted.linear.y = twist.linear.y;
            converted.linear.z = twist.linear.z;
            converted.angular.x = twist.angular.x;
            converted.angular.y = twist.angular.y;
            converted.angular.z = twist.angular.z;
            return converted;
        }
        inline geometry_msgs::msg::Twist ToRosMsg(const common::Twist & twist){
            geometry_msgs::msg::Twist converted;
            converted.linear.x = twist.linear.x;
            converted.linear.y = twist.linear.y;
            converted.linear.z = twist.linear.z;
            converted.angular.x = twist.angular.x;
            converted.angular.y = twist.angular.y;
            converted.angular.z = twist.angular.z;
            return converted;
        }
    }

        // Pose
        inline common::Pose FromRosMsg(const geometry_msgs::msg::Pose& pose) {
            common::Pose converted;
            converted.position.x = pose.position.x;
            converted.position.y = pose.position.y;
            converted.position.z = pose.position.z;
            converted.orientation.w = pose.orientation.w;
            converted.orientation.x = pose.orientation.x;
            converted.orientation.y = pose.orientation.y;
            converted.orientation.z = pose.orientation.z;

            return converted;
        }
        inline geometry_msgs::msg::Pose ToRosMsg(const common::Pose& pose) {
            geometry_msgs::msg::Pose converted;
            converted.position.x = pose.position.x;
            converted.position.y = pose.position.y;
            converted.position.z = pose.position.z;
            converted.orientation.w = pose.orientation.w;
            converted.orientation.x = pose.orientation.x;
            converted.orientation.y = pose.orientation.y;
            converted.orientation.z = pose.orientation.z;

            return converted;
        }

        // Header
        inline common::Header FromRosMsg(const std_msgs::msg::Header& header) {
            common::Header converted;
            converted.frame_id = header.frame_id;
            converted.nanosec = header.stamp.nanosec;
            converted.sec = header.stamp.sec;

            return converted;
        }
        inline std_msgs::msg::Header ToRosMsg(const common::Header& header) {
            std_msgs::msg::Header converted;
            converted.frame_id = header.frame_id;
            converted.stamp.set__nanosec(header.nanosec);
            converted.stamp.set__sec(header.sec);

            return converted;
        }

        // Pose stamp
        inline common::PoseStamped FromRosMsg(const geometry_msgs::msg::PoseStamped& pose_stamped) {
            common::PoseStamped converted;
            converted.header = FromRosMsg(pose_stamped.header);
            converted.pose = FromRosMsg(pose_stamped.pose);

            return converted;
        }
        inline geometry_msgs::msg::PoseStamped ToRosMsg(const common::PoseStamped& pose_stamped) {
            geometry_msgs::msg::PoseStamped converted;
            converted.header = ToRosMsg(pose_stamped.header);
            converted.pose = ToRosMsg(pose_stamped.pose);

            return converted;
        }

        // Odometry
        inline common::Odometry FromRosMsg(const nav_msgs::msg::Odometry & odometry) {
            common::Odometry converted;
            converted.pose = FromRosMsg(odometry.pose.pose);

            return converted;
        }
        inline nav_msgs::msg::Odometry ToRosMsg(const common::Odometry& odometry) {
            nav_msgs::msg::Odometry converted;
            converted.pose.pose = ToRosMsg(odometry.pose);

            return converted;
        }

        // Path
        inline common::Path FromRosMsg(const nav_msgs::msg::Path& path) {
            common::Path converted;
            for (const auto& p : path.poses) {
                converted.poses.push_back(FromRosMsg(p));
            }

            return converted;
        }
        inline nav_msgs::msg::Path ToRosMsg(const common::Path& path) {
            nav_msgs::msg::Path converted;
            for (const auto& p : path.poses) {
                converted.poses.push_back(ToRosMsg(p));
            }

            return converted;
        }

        // Occupancy grid
        inline common::OccupancyGrid FromRosMsg(const nav_msgs::msg::OccupancyGrid& occupancy_grid) {
            common::OccupancyGrid converted;
            converted.header = FromRosMsg(occupancy_grid.header);
            converted.info.resolution = occupancy_grid.info.resolution;
            converted.info.width = occupancy_grid.info.width;
            converted.info.height = occupancy_grid.info.height;
            converted.info.origin.position.x = occupancy_grid.info.origin.position.x;
            converted.info.origin.position.y = occupancy_grid.info.origin.position.y;
            converted.info.origin.position.z = occupancy_grid.info.origin.position.z;
            converted.info.origin.orientation.x = occupancy_grid.info.origin.orientation.x;
            converted.info.origin.orientation.y = occupancy_grid.info.origin.orientation.y;
            converted.info.origin.orientation.z = occupancy_grid.info.origin.orientation.z;
            converted.info.origin.orientation.w = occupancy_grid.info.origin.orientation.w;

            return converted;
        }
        inline nav_msgs::msg::OccupancyGrid ToRosMsg(const common::OccupancyGrid& occupancy_grid) {
            nav_msgs::msg::OccupancyGrid converted;
            common::OccupancyGrid converted;
            converted.header = ToRosMsg(occupancy_grid.header);
            converted.info.resolution = occupancy_grid.info.resolution;
            converted.info.width = occupancy_grid.info.width;
            converted.info.height = occupancy_grid.info.height;
            converted.info.origin.position.x = occupancy_grid.info.origin.position.x;
            converted.info.origin.position.y = occupancy_grid.info.origin.position.y;
            converted.info.origin.position.z = occupancy_grid.info.origin.position.z;
            converted.info.origin.orientation.x = occupancy_grid.info.origin.orientation.x;
            converted.info.origin.orientation.y = occupancy_grid.info.origin.orientation.y;
            converted.info.origin.orientation.z = occupancy_grid.info.origin.orientation.z;
            converted.info.origin.orientation.w = occupancy_grid.info.origin.orientation.w;

            return converted;
        }
}

#endif //NATURE_CONVERSION_H
