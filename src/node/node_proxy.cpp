#include "nature/node/node_proxy.h"

namespace nature {
namespace node {

#ifdef ROS_1

Rate::Rate(double hz) : rate_(hz) {
}

void Rate::sleep() {
    rate_.sleep();
}

NodeProxy::NodeProxy(const std::string &node_name) {
}

double NodeProxy::get_now_seconds() const {
    return get_stamp().toSec();
}

ros::Time NodeProxy::get_stamp() const {
    return ros::Time::now();
}

void NodeProxy::spin_some() {
    ros::spinOnce();
}

#else

    Rate::Rate(double hz) : rate_(hz) {
    }

    void Rate::sleep() {
      rate_.sleep();
    }

    NodeProxy::NodeProxy(const std::string &node_name) {
      node_ = rclcpp::Node::make_shared("nature_control_node");
      this->get_parameter("/is_empty_waypoints", is_empty_waypoints_, false);
    }

    rclcpp::Logger NodeProxy::get_logger() const {
      return node_->get_logger();
    }

    rclcpp::Time NodeProxy::get_stamp() const {
      return node_->get_clock()->now();
    }

    rclcpp::Clock::SharedPtr NodeProxy::get_clock() const {
      return node_->get_clock();
    }

    double NodeProxy::get_now_seconds() const {
      return get_stamp().seconds();
    }

    void NodeProxy::spin_some() {
      rclcpp::spin_some(node_);
    }

#endif


} // namespace node
} // namespace nature
