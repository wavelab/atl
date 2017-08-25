#include "atl/ros/utils/ros_topic_manager.hpp"

namespace atl {

int ROSTopicManager::registerShutdown(ros::NodeHandle &nh,
                                      const std::string &topic) {
  bool retval;
  ros::Subscriber sub;

  // register subscriber
  sub = nh.subscribe(topic, 1, &ROSTopicManager::shutdownCallback, this);
  this->ros_subs[topic] = sub;
}

int ROSTopicManager::registerImagePublisher(ros::NodeHandle &nh,
                                            const std::string &topic) {
  // image transport
  image_transport::ImageTransport it(nh);
  this->img_pubs[topic] = it.advertise(topic, 1);

  return 0;
}
} // namespace atl
