#ifdef ROS_1
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "nature_state_publisher");
  ros::NodeHandle n;
  tf::TransformBroadcaster broadcaster;

  boost::function<void(const nav_msgs::Odometry::ConstPtr& rcv_odom)> OdometryCallback = [&broadcaster](const nav_msgs::Odometry::ConstPtr& rcv_odom) -> void {
    std::cout<<"State publisher recieved odometry "<<std::endl;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.header.seq = rcv_odom->header.seq;
    odom_trans.header.stamp = rcv_odom->header.stamp;
    odom_trans.transform.translation.x = rcv_odom->pose.pose.position.x;
    odom_trans.transform.translation.y = rcv_odom->pose.pose.position.y;
    odom_trans.transform.translation.z = rcv_odom->pose.pose.position.z;
    odom_trans.transform.rotation = rcv_odom->pose.pose.orientation;
    broadcaster.sendTransform(odom_trans);
  };
  
  ros::Subscriber odom_sub = n.subscribe("nature/odometry",100, OdometryCallback);
  
  ros::spin();
  return 0;
}

#else 
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"

nav_msgs::msg::Odometry odometry;
geometry_msgs::msg::Pose &pose = odometry.pose.pose;
void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr rcv_odom){
  std::cout<<"State publisher recieved odometry "<<std::endl;
  odometry = *(rcv_odom.get());
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto n = rclcpp::Node::make_shared("nature_state_publisher");
  auto joint_pub = n->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  auto odom_sub = n->create_subscription<nav_msgs::msg::Odometry>("nature/odometry",100, OdometryCallback);

  tf2_ros::TransformBroadcaster broadcaster(n);

  // message declarations
  geometry_msgs::msg::TransformStamped odom_trans;
  sensor_msgs::msg::JointState joint_state;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  // set up parent and child frames
  geometry_msgs::msg::TransformStamped tf_map_to_odom;
  tf_map_to_odom.header.frame_id = std::string("map");
  tf_map_to_odom.child_frame_id = std::string("odom");

  rclcpp::Rate loop_rate(100.0);
  while (rclcpp::ok()) {
    //update joint_state
    joint_state.header.stamp = odometry.header.stamp;
    joint_state.name.resize(3);
    joint_state.position.resize(3);
    joint_state.name[0] ="lidar_joint";
    joint_state.position[0] = 0.0;

    odom_trans.header.stamp = odometry.header.stamp;
    odom_trans.transform.translation.x = pose.position.x;
    odom_trans.transform.translation.y = pose.position.y;
    odom_trans.transform.translation.z = pose.position.z;
    odom_trans.transform.rotation = pose.orientation;

    //send the joint state and transform
    joint_pub->publish(joint_state);
    broadcaster.sendTransform(odom_trans);

    // map to odom broadcast transform
    tf_map_to_odom.header.stamp = odometry.header.stamp;
    tf_map_to_odom.transform.translation.x = 0.0;
    tf_map_to_odom.transform.translation.y = 0.0;
    tf_map_to_odom.transform.translation.z = 0.0;
    tf_map_to_odom.transform.rotation.x = 0.0;
    tf_map_to_odom.transform.rotation.y = 0.0;
    tf_map_to_odom.transform.rotation.z = 0.0;
    tf_map_to_odom.transform.rotation.w = 1.0;

    broadcaster.sendTransform(tf_map_to_odom);

    // This will adjust as needed per iteration
    rclcpp::spin_some(n);
    loop_rate.sleep();
  }


  return 0;
}

#endif // ROS_1