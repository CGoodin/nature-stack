//
// Created by Stefan on 2021-07-28.
//

#ifndef NATURE_NODE_PROXY_H
#define NATURE_NODE_PROXY_H

#ifdef ROS_1

#include "ros/ros.h"
#include "std_msgs/Header.h"

namespace nature {
    namespace node {

        using Duration = ros::Duration;

        inline Duration make_duration(float period){
            float sec;
            float fraction = std::modf(period, & sec);
            return Duration(static_cast<int32_t>(sec), static_cast<int32_t>(fraction * 1e9));
        }

        inline Duration make_duration(int32_t sec, int32_t nsec){
            return ros::Duration(sec, nsec);
        }

        template<typename MessageT>
        class Publisher {
        public:
            explicit Publisher(const std::string &topic_name, int qos, ros::NodeHandle & node) {
                pub_ = node.advertise<MessageT>(topic_name, qos);
            }
            Publisher() = default;

            void publish(const MessageT &msg) {
                pub_.publish(msg);
            }

        private:
            ros::Publisher pub_;
        };

        template<
                typename MessageT>
        class Subscriber {

        public:
            Subscriber(const std::string & topic_name, uint qos, void(*callback)(const boost::shared_ptr<MessageT const>&), ros::NodeHandle & node) {
                sub_ptr_ = node.subscribe<MessageT>(topic_name, qos, callback);
            }

        private:
            ros::Subscriber sub_ptr_;
        };

        inline double seconds_from_header(std_msgs::Header header){
            return header.stamp.toSec();
        }

        inline ros::Time time_from_seconds(double sec){
            return ros::Time(sec);
        }

        inline void inc_seq(std_msgs::Header & header){
          header.seq++;
        }

        inline void set_seq(std_msgs::Header & header, int seq){
          header.seq = seq;
        }

        inline bool ok() {
            return ros::ok();
        }

        inline void init(int argc, char *argv[], const std::string & node_name) {
            ros::init(argc,argv,node_name);
        }

        class Rate {

        public:
            Rate(double hz);
            void sleep();
        private:
            ros::Rate rate_;
        };

        class NodeProxy {

        public:

            NodeProxy(const std::string &node_name);

            template<typename ParameterT>
            bool get_parameter(const std::string &name, ParameterT &parameter_out, const ParameterT default_value) {
                if (ros::param::has(name)){
                    ros::param::get(name, parameter_out);
                    return true;
                }else{
                    parameter_out = default_value;
                    return false;
                }
            }

            template<typename MessageT>
            std::shared_ptr<Publisher<MessageT>> create_publisher(const std::string &topic_name, int qos) {
                return std::make_shared<Publisher<MessageT>>(topic_name, qos, node_);
            }

            template<typename MessageT>
            std::shared_ptr<Subscriber<MessageT>> create_subscription(const std::string &topic_name, uint qos, void(*callback)(const boost::shared_ptr<MessageT const>&)) {
                return std::make_shared<Subscriber<MessageT>>(topic_name, qos, callback, node_);
            }

            ros::Time get_stamp() const;
            double get_now_seconds() const;
            void spin_some();

        private:
            ros::NodeHandle node_;
        };

        inline std::shared_ptr<NodeProxy> make_shared(const std::string &name) {
            return std::make_shared<NodeProxy>(name);
        }

        inline std::shared_ptr<NodeProxy> init_node(int argc, char *argv[], const std::string &name){
            init(argc, argv, name);
            return make_shared(name);
        }
    }
}

#else


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace nature {
  namespace node {

    using Duration = rclcpp::Duration;

        inline Duration make_duration(float period){
            float sec;
            float fraction = std::modf(period, & sec);
            return Duration(static_cast<int32_t>(sec), static_cast<int32_t>(fraction * 1e9));
        }

        inline Duration make_duration(int32_t sec, int32_t nsec){
            return rclcpp::Duration(sec, nsec);
        }

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

    inline rclcpp::Time time_from_seconds(double sec){
            return rclcpp::Time(sec);
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

        if(is_empty_waypoints_ && (name_local == "/waypoints_x" || name_local == "/waypoints_y")){
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
      rclcpp::Clock::SharedPtr get_clock() const;
      void spin_some();

    private:
      std::shared_ptr<rclcpp::Node> node_;
      bool is_empty_waypoints_;
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

#endif //ROS_1

#endif //NATURE_NODE_PROXY_H