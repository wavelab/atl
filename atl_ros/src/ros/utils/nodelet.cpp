#include "atl/ros/utils/nodelet.hpp"

namespace atl {

void ROSNodelet::configureNodelet(int hz) {
  this->nh = getNodeHandle();
  this->pnh = getPrivateNodeHandle();

  double period = 1.0 / hz;

  this->timer = nh.createTimer(ros::Duration(period),
                               boost::bind(&ROSNodelet::loop_cb, this));
}

int ROSNodelet::registerShutdown(const std::string &topic) {
  bool retval;
  ros::Subscriber sub;

  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // register subscriber
  sub = this->ros_nh->subscribe(topic, 1, &ROSNode::shutdownCallback, this);
  this->ros_subs[topic] = sub;
}

int ROSNodelet::registerImagePublisher(const std::string &topic) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // image transport
  image_transport::ImageTransport it(this->nh);
  this->img_pubs[topic] = it.advertise(topic, 1);

  return 0;
}

int ROSNodelet::registerLoopCallback(std::function<int()> cb) {
  this->loop_cb = cb;
  return 0;
}
}