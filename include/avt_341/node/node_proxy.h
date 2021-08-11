//
// Created by Stefan on 2021-07-28.
//

#ifndef AVT_341_NODE_PROXY_H
#define AVT_341_NODE_PROXY_H

#include "ros/ros.h"
#include "std_msgs/Header.h"

namespace avt_341 {
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
                }else{
                    parameter_out = default_value;
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

#endif //AVT_341_NODE_PROXY_H