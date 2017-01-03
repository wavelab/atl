#ifndef __AWESOMO_ROS_ROS_NODE_HPP__
#define __AWESOMO_ROS_ROS_NODE_HPP__

#include <functional>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

namespace awesomo {

class ROSNode {
public:
  bool configured;
  bool debug_mode;
  bool sim_mode;

  long long int ros_seq;
  ::ros::NodeHandle *ros_nh;
  ::ros::Rate *ros_rate;
  ::ros::Time ros_last_updated;
  std::map<std::string, std::string> ros_topics;
  std::map<std::string, ::ros::Publisher> ros_pubs;
  std::map<std::string, ::ros::Subscriber> ros_subs;
  image_transport::Publisher img_pub;
  std::function<int(void)> loop_cb;

  ROSNode(void) {
    this->configured = false;
    this->debug_mode = false;
    this->sim_mode = false;
    this->ros_seq = 0;
    this->ros_rate = NULL;
  }

  ~ROSNode(void) {
    ::ros::shutdown();
  }

  int configure(const std::string node_name, int hz) {
    // clang-format off
    if (::ros::isInitialized() == false) {
      int argc = 0;
      char **argv = NULL;
      ::ros::init(argc,
        argv,
        node_name,
        ::ros::init_options::NoSigintHandler
      );
    }
    // clang-format on

    this->ros_nh = new ::ros::NodeHandle();
    this->ros_nh->getParam("/debug_mode", this->debug_mode);
    this->ros_nh->getParam("/sim_mode", this->sim_mode);
    this->ros_rate = new ::ros::Rate(hz);
    this->configured = true;

    return 0;
  }

  int registerTopic(std::string topic_name, std::string &topic_url) {
    bool retval;

    retval = this->ros_nh->getParam(topic_name, topic_url);
    if (retval) {
      this->ros_topics[topic_name] = topic_url;
      return 0;
    } else {
      return -1;
    }
  }

  int registerImagePublisher(const std::string &topic_name) {
    std::string topic_url;

    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // register topic
    if (this->registerTopic(topic_name, topic_url) != 0) {
      ROS_ERROR("TOPIC [%s] not found in launch file!", topic_name.c_str());
      return -2;
    }

    // image transport
    image_transport::ImageTransport it(*this->ros_nh);
    this->img_pub = it.advertise(topic_url, 100);

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
      ROS_ERROR("TOPIC [%s] not found in launch file!", topic_name.c_str());
      return -2;
    }

    // register publisher
    publisher = this->ros_nh->advertise<M>(topic_url, queue_size, latch);
    ROS_INFO("Publisher [%s] initialized!", topic_url.c_str());
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
      ROS_ERROR("TOPIC [%s] not found in launch file!", topic_name.c_str());
      return -2;
    }

    // register subscriber
    ROS_INFO("Subscriber [%s] initialized!", topic_url.c_str());
    subscriber = this->ros_nh->subscribe(topic_url, queue_size, fp, obj);
    this->ros_subs[topic_name] = subscriber;

    return 0;
  }

  int registerLoopCallback(std::function<int(void)> cb) {
    this->loop_cb = cb;
    return 0;
  }

  int loop(void) {
    int retval;

    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // print mode
    if (this->debug_mode) {
      ROS_INFO("Running debug mode!");
    }
    if (this->sim_mode) {
      ROS_INFO("Running sim mode!");
    }

    // loop
    while (::ros::ok()) {
      // run loop callback
      if (this->loop_cb != nullptr) {
        retval = this->loop_cb();
        if (retval != 0) {
          return retval;
        }
      }

      // update
      ::ros::spinOnce();
      this->ros_seq++;
      this->ros_last_updated = ::ros::Time::now();
      this->ros_rate->sleep();
    }

    return 0;
  }
};

}  // end of awesomo namespace
#endif
