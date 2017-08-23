#ifndef ATL_ROS_NODELET_HPP
#define ATL_ROS_NODELET_HPP

#include <functional>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>

namespace atl {

class ROSNodelet : public nodelet::Nodelet {
public:
  ROSNodelet() {}

  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Timer timer;

  ros::Time ros_last_updated;

  std::map<std::string, ros::Publisher> ros_pubs;
  std::map<std::string, ros::Subscriber> ros_subs;
  std::map<std::string, ros::ServiceServer> ros_servers;
  std::map<std::string, ros::ServiceClient> ros_clients;
  std::map<std::string, image_transport::Publisher> img_pubs;
  std::map<std::string, image_transport::Subscriber> img_subs;

  bool configured = false;

  void configureNodelet(int hz);

  int registerShutdown(const std::string &topic);
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
    image_transport::ImageTransport it(this->nh);
    this->img_subs[topic] = it.subscribe(topic, queue_size, fp, obj);

    return 0;
  }

  template <typename M>
  int registerPublisher(const std::string &topic,
                        uint32_t queue_size = 1,
                        bool latch = false) {
    ros::Publisher publisher;

    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // register publisher
    publisher = this->nh.advertise<M>(topic, queue_size, latch);
    this->ros_pubs[topic] = publisher;

    return 0;
  }

  template <typename M, typename T>
  int registerSubscriber(const std::string &topic,
                         void (T::*fp)(M),
                         T *obj,
                         uint32_t queue_size = 1) {
    ros::Subscriber subscriber;

    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // register subscriber
    subscriber = this->nh.subscribe(topic, queue_size, fp, obj);
    this->ros_subs[topic] = subscriber;

    return 0;
  }

  template <class T, class MReq, class MRes>
  int registerServer(const std::string &service_topic,
                     bool (T::*fp)(MReq &, MRes &),
                     T *obj) {
    ros::ServiceServer server;

    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // register service server
    server = this->nh.advertiseService(service_topic, fp, obj);
    this->ros_servers[service_topic] = server;

    return 0;
  }

  template <typename M>
  int registerClient(const std::string &service_topic,
                     bool persistent = false) {
    ros::ServiceClient client;

    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // register service server
    client = this->nh.serviceClient<M>(service_topic);
    this->ros_clients[service_topic] = client;

    return 0;
  }

  int registerLoopCallback(std::function<int()> cb);
  std::function<int()> loop_cb;

  void shutdownCallback(const std_msgs::BoolConstPtr &msg) {
    if (msg->data) {
      ros::shutdown();
    }
  }
};
} // namespace atl

#endif // ATL_ROS_NODELET_HPP
