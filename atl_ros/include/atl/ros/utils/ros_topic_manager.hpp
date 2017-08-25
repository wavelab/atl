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

class ROSTopicManager {
public:
  ROSTopicManager() = default;

  ~ROSTopicManager() = default;

  int registerShutdown(ros::NodeHandle &nh, const std::string &topic);
  int registerImagePublisher(ros::NodeHandle &nh, const std::string &topic);

  template <typename M, typename T>
  int registerImageSubscriber(ros::NodeHandle &nh,
                              const std::string &topic,
                              void (T::*fp)(M),
                              T *obj,
                              uint32_t queue_size = 1) {

    // image transport
    image_transport::ImageTransport it(nh);
    this->img_subs[topic] = it.subscribe(topic, queue_size, fp, obj);

    return 0;
  }

  template <typename M>
  int registerPublisher(ros::NodeHandle &nh,
                        const std::string &topic,
                        uint32_t queue_size = 1,
                        bool latch = false) {
    ros::Publisher publisher;

    // register publisher
    publisher = nh.advertise<M>(topic, queue_size, latch);
    this->ros_pubs[topic] = publisher;

    return 0;
  }

  template <typename M, typename T>
  int registerSubscriber(ros::NodeHandle &nh,
                         const std::string &topic,
                         void (T::*fp)(M),
                         T *obj,
                         uint32_t queue_size = 1) {
    ros::Subscriber subscriber;

    // register subscriber
    subscriber = nh.subscribe(topic, queue_size, fp, obj);
    this->ros_subs[topic] = subscriber;

    return 0;
  }

  template <class T, class MReq, class MRes>
  int registerServer(ros::NodeHandle &nh,
                     const std::string &service_topic,
                     bool (T::*fp)(MReq &, MRes &),
                     T *obj) {
    ros::ServiceServer server;

    // register service server
    server = nh.advertiseService(service_topic, fp, obj);
    this->ros_servers[service_topic] = server;

    return 0;
  }

  template <typename M>
  int registerClient(ros::NodeHandle &nh,
                     const std::string &service_topic,
                     bool persistent = false) {
    ros::ServiceClient client;

    // register service server
    client = nh.serviceClient<M>(service_topic);
    this->ros_clients[service_topic] = client;

    return 0;
  }

  void shutdownCallback(const std_msgs::BoolConstPtr &msg) {
    if (msg->data) {
      ros::shutdown();
    }
  }

  std::map<std::string, ros::Publisher> ros_pubs;
  std::map<std::string, ros::Subscriber> ros_subs;
  std::map<std::string, ros::ServiceServer> ros_servers;
  std::map<std::string, ros::ServiceClient> ros_clients;
  std::map<std::string, image_transport::Publisher> img_pubs;
  std::map<std::string, image_transport::Subscriber> img_subs;
};
} // namespace atl

#endif // ATL_ROS_NODELET_HPP
