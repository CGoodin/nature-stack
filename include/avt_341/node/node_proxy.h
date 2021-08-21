//
// Created by Stefan on 2021-07-28.
//

#ifndef AVT_341_NODE_PROXY_H
#define AVT_341_NODE_PROXY_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace avt_341 {
  namespace node {
    template<
        typename MessageT,
        typename AllocatorT = std::allocator<void>,
        typename PublisherT = rclcpp::Publisher<MessageT, AllocatorT>>
    class Publisher {
    public:
      explicit Publisher(const std::string &topic_name, int qos, const std::shared_ptr<rclcpp::Node> &node_) {
        pub_ptr_ = node_->create_publisher<MessageT>(topic_name, qos);
      }

      void publish(const MessageT &msg) {
        pub_ptr_->publish(msg);
      }

    private:
      std::shared_ptr<PublisherT> pub_ptr_;
    };

    template<
        typename MessageT,
        typename CallbackT,
        typename AllocatorT = std::allocator<void>,
        typename CallbackMessageT = typename rclcpp::subscription_traits::has_message_type<CallbackT>::type>
    class Subscriber {

    public:
      Subscriber(const std::string &topic_name, int qos, CallbackT &&callback,
                 const std::shared_ptr<rclcpp::Node> &node_) {
        sub_ptr_ = node_->create_subscription<MessageT>(topic_name, qos, callback);
      }

    private:
      std::shared_ptr<rclcpp::Subscription<CallbackMessageT, AllocatorT>> sub_ptr_;
    };

    inline double seconds_from_header(const std_msgs::msg::Header & header){
      return rclcpp::Time(header.stamp).seconds();
    }

    inline void inc_seq(const std_msgs::msg::Header & header){
      // Seq removed from ROS2
    }

    inline void set_seq(const std_msgs::msg::Header & header, int seq){
      // Seq removed from ROS2
    }

    inline bool ok() {
      return rclcpp::ok();
    }

    inline void init(int argc, char *argv[]) {
      rclcpp::init(argc, argv);
    }

    class Rate {

    public:
      Rate(double hz);
      void sleep();
    private:
      rclcpp::Rate rate_;
    };

    class NodeProxy {

    public:

      NodeProxy(const std::string &node_name);

      template<typename ParameterT>
      void get_parameter(const std::string &name, ParameterT &parameter_out, const ParameterT default_value) {
        std::string name_local = name[0] == '~' ? name.substr(1, name.size()-1) : name;

        if(name_local == "use_sim_time"){
          // TODO: Currently not supported
          parameter_out = default_value;
          return;
        }

        node_->declare_parameter(name_local, default_value);
        node_->get_parameter(name_local, parameter_out);
      }

      template<typename MessageT>
      std::shared_ptr<Publisher<MessageT>> create_publisher(const std::string &topic_name, int qos) {
        return std::make_shared<Publisher<MessageT>>(topic_name, qos, node_);
      }

      template<typename MessageT, typename CallbackT>
      std::shared_ptr<Subscriber<MessageT, CallbackT>>
      create_subscription(const std::string &topic_name, int qos, CallbackT &&callback) {
        return std::make_shared<Subscriber<MessageT, CallbackT>>(topic_name, qos, callback, node_);
      }

      rclcpp::Logger get_logger() const;
      rclcpp::Time get_stamp() const;
      double get_now_seconds() const;
      void spin_some();

    private:
      std::shared_ptr<rclcpp::Node> node_;
    };

    inline std::shared_ptr<NodeProxy> make_shared(const std::string &name) {
      return std::make_shared<NodeProxy>(name);
    }

    inline std::shared_ptr<NodeProxy> init_node(int argc, char *argv[], const std::string &name){
      init(argc, argv);
      return make_shared(name);
    }
  }
}

#endif //AVT_341_NODE_PROXY_H
