#ifndef atl_GAZEBO_NODE_HPP
#define atl_GAZEBO_NODE_HPP

#include <string>
#include <vector>

#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include "atl/gazebo/msgs/atl_msgs.hpp"

namespace atl {
namespace gaz {

class GazeboNode {
public:
  bool configured;
  gazebo::transport::NodePtr gaz_node;
  std::map<std::string, gazebo::transport::PublisherPtr> gaz_pubs;
  std::map<std::string, gazebo::transport::SubscriberPtr> gaz_subs;

  GazeboNode() : configured{false} {}

  int configure() {
    this->gaz_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gaz_node->Init();
    this->configured = true;

    return 0;
  }

  int configure(std::string world_name) {
    this->gaz_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gaz_node->Init(world_name);
    this->configured = true;

    return 0;
  }

  template <typename M>
  int addPublisher(const std::string topic,
                   unsigned int queue_limit = 1000,
                   double rate = 0.0) {
    gazebo::transport::PublisherPtr pub_ptr;

    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // publish
    pub_ptr = this->gaz_node->Advertise<M>(topic, queue_limit, rate);
    this->gaz_pubs[topic] = pub_ptr;

    return 0;
  }

  int waitForConnection() {
    gazebo::transport::PublisherPtr pub;
    std::map<std::string, gazebo::transport::PublisherPtr>::iterator it;

    it = this->gaz_pubs.begin();
    while (it != this->gaz_pubs.end()) {
      pub = it->second;
      pub->WaitForConnection();
      it++;
    }

    return 0;
  }

  template <typename M, typename T>
  int addSubscriber(std::string topic,
                    void (T::*fp)(const boost::shared_ptr<M const> &),
                    T *obj) {
    gazebo::transport::SubscriberPtr sub_ptr;

    // pre-check
    if (this->configured == false) {
      return -1;
    }

    // subscribe
    sub_ptr = this->gaz_node->Subscribe(topic, fp, obj);
    this->gaz_subs[topic] = sub_ptr;

    return 0;
  }
};

} // namespace gaz
} // namespace atl
#endif
