#include "nature/node/clock_publisher.h"

#ifdef ROS_1

std::shared_ptr<nature::node::ClockPublisher>
nature::node::ClockPublisher::make_shared(const std::string &topic_name, int qos, std::shared_ptr<nature::node::NodeProxy> node) {
  return std::make_shared<nature::node::ClockPublisher>(topic_name, qos, node);
}

nature::node::ClockPublisher::ClockPublisher(const std::string topic_name, int qos, std::shared_ptr<nature::node::NodeProxy> node) {
    pub_ = node->create_publisher<rosgraph_msgs::Clock>("clock", 1);
}

void nature::node::ClockPublisher::publish(double elapsed_time) {
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = nature::node::time_from_seconds(elapsed_time);
    pub_->publish(clock_msg);
}

#else

std::shared_ptr<nature::node::ClockPublisher>
nature::node::ClockPublisher::make_shared(const std::string &topic_name, int qos, std::shared_ptr<nature::node::NodeProxy> node) {
  return std::make_shared<nature::node::ClockPublisher>(topic_name, qos, node);
}

nature::node::ClockPublisher::ClockPublisher(const std::string topic_name, int qos, std::shared_ptr<nature::node::NodeProxy> node) {
}

void nature::node::ClockPublisher::publish(double elapsed_time) {
  // Not currently supported on ROS2 branch
}

#endif // ROS_1