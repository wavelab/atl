#ifndef ATL_ROS_UTILS_NODE_HPP
#define ATL_ROS_UTILS_NODE_HPP

#include <functional>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>

namespace atl {

#define INFO_CONFIG "Configuring ROS Node [%s]!"
#define ROS_GET_PARAM(X, Y)                                                    \
  if (this->ros_nh == NULL) {                                                  \
    ROS_ERROR("You did not do ROSNode::configure() first!");                   \
    ROS_ERROR("Can only call ROS_GET_PARAM() after configure!");               \
    return -1;                                                                 \
  }                                                                            \
  if (this->ros_nh->getParam(X, Y) == false) {                                 \
    ROS_ERROR("Failed to get ROS param [%s]", #X);                             \
    return -1;                                                                 \
  }
#define RUN_ROS_NODE(NODE_CLASS, NODE_RATE)                                    \
  int main(int argc, char **argv) {                                            \
    NODE_CLASS node(argc, argv);                                               \
    if (node.configure(NODE_RATE) != 0) {                                      \
      ROS_ERROR("Failed to configure %s", #NODE_CLASS);                        \
      return -1;                                                               \
    }                                                                          \
    node.loop();                                                               \
    return 0;                                                                  \
  }

class ROSNode {
public:
  bool configured;
  bool debug_mode;
  bool sim_mode;

  int argc;
  char **argv;

  std::string node_name;
  long long int ros_seq;
  ros::NodeHandle *ros_nh;
  ros::Rate *ros_rate;
  ros::Time ros_last_updated;

  std::map<std::string, ros::Publisher> ros_pubs;
  std::map<std::string, ros::Subscriber> ros_subs;
  std::map<std::string, ros::ServiceServer> ros_servers;
  std::map<std::string, ros::ServiceClient> ros_clients;

  std::map<std::string, image_transport::Publisher> img_pubs;
  std::map<std::string, image_transport::Subscriber> img_subs;
  std::function<int()> loop_cb;

  ROSNode() {}
  ROSNode(int argc, char **argv);
  ~ROSNode();
  int configure(int hz);
  void shutdownCallback(const std_msgs::Bool &msg);
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
    image_transport::ImageTransport it(*this->ros_nh);
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
    publisher = this->ros_nh->advertise<M>(topic, queue_size, latch);
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
    subscriber = this->ros_nh->subscribe(topic, queue_size, fp, obj);
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
    server = this->ros_nh->advertiseService(service_topic, fp, obj);
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
    client = this->ros_nh->serviceClient<M>(service_topic);
    this->ros_clients[service_topic] = client;

    return 0;
  }

  int registerLoopCallback(std::function<int()> cb);
  int loop();
};

} // namespace atl
#endif
