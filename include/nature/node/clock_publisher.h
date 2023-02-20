#ifndef CLOCK_PUBLISHER_H
#define CLOCK_PUBLISHER_H

#ifdef ROS_1

#include "nature/node/node_proxy.h"
#include "rosgraph_msgs/Clock.h"

namespace nature {
    namespace node {

        class ClockPublisher{
            public:
                static std::shared_ptr<ClockPublisher> make_shared(const std::string & topic_name, int qos, std::shared_ptr<NodeProxy> node);
                ClockPublisher(const std::string topic_name, int qos, std::shared_ptr<NodeProxy> node);
                void publish(double elapsed_time);
            private:
                std::shared_ptr<nature::node::Publisher<rosgraph_msgs::Clock>> pub_;
        };

    }
}

#else

#include "nature/node/node_proxy.h"

namespace nature {
  namespace node {

    class ClockPublisher{
    public:
      static std::shared_ptr<ClockPublisher> make_shared(const std::string & topic_name, int qos, std::shared_ptr<NodeProxy> node);
      ClockPublisher(const std::string topic_name, int qos, std::shared_ptr<NodeProxy> node);
      void publish(double elapsed_time);
    private:
    };

  }
}


#endif // ROS_1

#endif //CLOCK_PUBLISHER_H
