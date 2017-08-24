#include "atl/ros/utils/nodelet_helper.hpp"

namespace atl {

void NodeletHelper::configure(int hz, ros::NodeHandle nh) {
  this->nh = nh;

  double period = 1.0 / hz;

  this->timer = nh.createTimer(ros::Duration(period),
                               boost::bind(&NodeletHelper::loop_cb, this));

  this->configured = true;
}

int NodeletHelper::registerShutdown(const std::string &topic) {
  bool retval;
  ros::Subscriber sub;

  // pre-check
  if (!this->configured) {
    return -1;
  }

  // register subscriber
  sub = this->nh.subscribe(topic, 1, &NodeletHelper::shutdownCallback, this);
  this->ros_subs[topic] = sub;
}

int NodeletHelper::registerImagePublisher(const std::string &topic) {
  // pre-check
  if (!this->configured) {
    return -1;
  }

  // image transport
  image_transport::ImageTransport it(this->nh);
  this->img_pubs[topic] = it.advertise(topic, 1);

  return 0;
}

int NodeletHelper::registerLoopCallback(std::function<int()> cb) {
  this->loop_cb = cb;
  return 0;
}
} // namespace atl

