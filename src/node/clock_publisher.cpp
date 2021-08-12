#include "avt_341/node/clock_publisher.h"

std::shared_ptr<avt_341::node::ClockPublisher>
avt_341::node::ClockPublisher::make_shared(const std::string &topic_name, int qos, std::shared_ptr<avt_341::node::NodeProxy> node) {
  return std::make_shared<avt_341::node::ClockPublisher>(topic_name, qos, node);
}

avt_341::node::ClockPublisher::ClockPublisher(const std::string topic_name, int qos, std::shared_ptr<avt_341::node::NodeProxy> node) {
    pub_ = node->create_publisher<rosgraph_msgs::Clock>("clock", 1);
}

void avt_341::node::ClockPublisher::publish(double elapsed_time) {
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = avt_341::node::time_from_seconds(elapsed_time);
    pub_->publish(clock_msg);
}
