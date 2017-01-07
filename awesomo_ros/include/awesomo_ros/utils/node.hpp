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
#define INFO_PUB_INIT "--> Publisher [%s] initialized!"
#define INFO_SUB_INIT "--> Subscriber [%s] initialized!"
#define INFO_DEBUG_MODE "ROS Node [%s] is running in [DEBUG MODE]!"
#define INFO_SIM_MODE "ROS Node [%s] is running in [SIM MODE]!"
#define INFO_NORMAL_MODE "ROS Node [%s] is running..."

#define E_TOPIC "TOPIC [%s] not found in launch file!"
#define E_SHUTDOWN_TOPIC "SHUTDOWN TOPIC [%s] not found in launch file!"

class ROSNode {
public:
  bool configured;
  bool debug_mode;
  bool sim_mode;

  std::string ros_node_name;
  long long int ros_seq;
  ::ros::NodeHandle *ros_nh;
  ::ros::Rate *ros_rate;
  ::ros::Time ros_last_updated;

  std::map<std::string, std::string> ros_topics;
  std::map<std::string, ::ros::Publisher> ros_pubs;
  std::map<std::string, ::ros::Subscriber> ros_subs;

  image_transport::Publisher img_pub;
  image_transport::Subscriber img_sub;
  std::function<int(void)> loop_cb;

  ROSNode(void);
  ~ROSNode(void);
  int configure(const std::string node_name, int hz);
  void shutdownCallback(const std_msgs::Bool &msg);
  int registerTopic(std::string topic_name, std::string &topic_url);
  int registerShutdown(std::string topic_name);
  int registerImagePublisher(const std::string &topic_name);

  template <typename M, typename T>
  int registerImageSubscriber(const std::string &topic_name,
                              void (T::*fp)(M),
                              T *obj,
                              uint32_t queue_size = 1) {
    std::string topic_url;

    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // register topic
    if (this->registerTopic(topic_name, topic_url) != 0) {
      ROS_ERROR(E_TOPIC, topic_name.c_str());
      return -2;
    }

    // image transport
    image_transport::ImageTransport it(*this->ros_nh);
    this->img_sub = it.subscribe(topic_url, queue_size, fp, obj);

    return 0;
  }

  template <typename M>
  int registerPublisher(const std::string &topic_name,
                        uint32_t queue_size = 100,
                        bool latch = false) {
    std::string topic_url;
    ::ros::Publisher publisher;

    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // register topic
    if (this->registerTopic(topic_name, topic_url) != 0) {
      ROS_ERROR(E_TOPIC, topic_name.c_str());
      return -2;
    }

    // register publisher
    publisher = this->ros_nh->advertise<M>(topic_url, queue_size, latch);
    ROS_INFO(INFO_PUB_INIT, topic_url.c_str());
    this->ros_pubs[topic_name] = publisher;

    return 0;
  }

  template <typename M, typename T>
  int registerSubscriber(const std::string &topic_name,
                         void (T::*fp)(M),
                         T *obj,
                         uint32_t queue_size = 100) {
    std::string topic_url;
    ::ros::Subscriber subscriber;

    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // register topic
    if (this->registerTopic(topic_name, topic_url) != 0) {
      ROS_ERROR(E_TOPIC, topic_name.c_str());
      return -2;
    }

    // register subscriber
    ROS_INFO(INFO_SUB_INIT, topic_url.c_str());
    subscriber = this->ros_nh->subscribe(topic_url, queue_size, fp, obj);
    this->ros_subs[topic_name] = subscriber;

    return 0;
  }

  int registerLoopCallback(std::function<int(void)> cb);
  int loop(void);
};

}  // end of awesomo namespace
#endif
