#include "avt_341/node/node_proxy.h"

namespace avt_341 {
namespace node {

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

}
}
