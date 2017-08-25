#include "atl/ros/utils/ros_topic_manager.hpp"

namespace atl {

void ROSTopicManager::configure(ros::NodeHandle nh) {
  this->nh = nh;
  this->configured = true;
}

int ROSTopicManager::registerShutdown(const std::string &topic) {
  bool retval;
  ros::Subscriber sub;

  // pre-check
  if (!this->configured) {
    return -1;
  }

  // register subscriber
  sub = this->nh.subscribe(topic, 1, &ROSTopicManager::shutdownCallback, this);
  this->ros_subs[topic] = sub;
}

int ROSTopicManager::registerImagePublisher(const std::string &topic) {
  // pre-check
  if (!this->configured) {
    return -1;
  }

  // image transport
  image_transport::ImageTransport it(this->nh);
  this->img_pubs[topic] = it.advertise(topic, 1);

  return 0;
}

int ROSTopicManager::registerLoopCallback(std::function<int()> cb) {
  this->loop_cb = cb;

  return 0;
}
} // namespace atl
