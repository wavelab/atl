#ifndef __AWESOMO_ROS_UTILS_NODE_HPP__
#define __AWESOMO_ROS_UTILS_NODE_HPP__

#include <functional>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

namespace awesomo {

#define INFO_CONFIG "Configuring ROS Node [%s]!"

class ROSNode {
public:
  bool configured;
  bool debug_mode;
  bool sim_mode;

  int argc;
  char **argv;

  std::string ros_node_name;
  long long int ros_seq;
  ::ros::NodeHandle *ros_nh;
  ::ros::Rate *ros_rate;
  ::ros::Time ros_last_updated;

  std::map<std::string, ::ros::Publisher> ros_pubs;
  std::map<std::string, ::ros::Subscriber> ros_subs;

  image_transport::Publisher img_pub;
  image_transport::Subscriber img_sub;
  std::function<int(void)> loop_cb;

  ROSNode(int argc, char **argv);
  ~ROSNode(void);
  int configure(const std::string node_name, int hz);
  void shutdownCallback(const std_msgs::Bool &msg);
  int registerShutdown(std::string topic);
  int registerImagePublisher(const std::string &topic);

  template <typename M, typename T>
  int registerImageSubscriber(const std::string &topic,
                              void (T::*fp)(M),
                              T *obj,
                              uint32_t queue_size = 1) {
    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // image transport
    image_transport::ImageTransport it(*this->ros_nh);
    this->img_sub = it.subscribe(topic, queue_size, fp, obj);

    return 0;
  }

  template <typename M>
  int registerPublisher(const std::string &topic,
                        uint32_t queue_size = 1,
                        bool latch = false) {
    ::ros::Publisher publisher;

    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // register publisher
    publisher = this->ros_nh->advertise<M>(topic, queue_size, latch);
    this->ros_pubs[topic] = publisher;

    return 0;
  }

  template <typename M, typename T>
  int registerSubscriber(const std::string &topic,
                         void (T::*fp)(M),
                         T *obj,
                         uint32_t queue_size = 1) {
    ::ros::Subscriber subscriber;

    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // register subscriber
    subscriber = this->ros_nh->subscribe(topic, queue_size, fp, obj);
    this->ros_subs[topic] = subscriber;

    return 0;
  }

  int registerLoopCallback(std::function<int(void)> cb);
  int loop(void);
};

}  // end of awesomo namespace
#endif
