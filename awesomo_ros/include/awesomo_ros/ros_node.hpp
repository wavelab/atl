#ifndef __AWESOMO_ROS_ROS_NODE_HPP__
#define __AWESOMO_ROS_ROS_NODE_HPP__

#include <functional>

#include <ros/ros.h>


namespace awesomo {

class ROSNode {
public:
  bool configured;
  bool debug_mode;

  long long int ros_seq;
  ::ros::NodeHandle *ros_nh;
  ::ros::Rate *ros_rate;
  ::ros::Time ros_last_updated;
  std::map<std::string, ::ros::Publisher> ros_pubs;
  std::map<std::string, ::ros::Subscriber> ros_subs;
  std::function<int(void)> loop_cb;

  ROSNode(void) {
    this->configured = false;
    this->debug_mode = false;
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
    this->ros_nh->getParam("/debug", this->debug_mode);
    this->ros_rate = new ::ros::Rate(hz);
    this->configured = true;

    return 0;
  }

  template <typename M>
  int registerPublisher(const std::string &topic,
                        uint32_t queue_size = 100,
                        bool latch = false) {
    ::ros::Publisher publisher;

    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // register publisher
    publisher = this->ros_nh->advertise<M>(topic, queue_size, latch);
    ROS_INFO("Publisher [%s] initialized!", topic.c_str());
    this->ros_pubs[topic] = publisher;

    return 0;
  }

  template <typename M, typename T>
  int registerSubscriber(const std::string &topic,
                         void (T::*fp)(M),
                         T *obj,
                         uint32_t queue_size = 100) {
    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // register subscriber
    ::ros::Subscriber subscriber;
    ROS_INFO("Subscriber [%s] initialized!", topic.c_str());
    subscriber = this->ros_nh->subscribe(topic, queue_size, fp, obj);
    this->ros_subs[topic] = subscriber;

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
