#ifndef CLOCK_PUBLISHER_H
#define CLOCK_PUBLISHER_H

#include "avt_341/node/node_proxy.h"

namespace avt_341 {
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
#endif //CLOCK_PUBLISHER_H
