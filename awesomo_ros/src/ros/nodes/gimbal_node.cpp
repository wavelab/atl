#include "awesomo_ros/nodes/gimbal_node.hpp"

namespace awesomo {

int GimbalNode::configure(std::string node_name, int hz) {
  std::string config_file;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // gimbal
  this->ros_nh->getParam("/gimbal_config", config_file);
  if (this->gimbal.configure(config_file) != 0) {
    ROS_ERROR("Failed to configure Gimbal!");
    return -2;
  };

  // register publisher and subscribers
  // clang-format off
  this->registerSubscriber(TRACK_TOPIC, &GimbalNode::trackTargetCallback, this);
  this->registerSubscriber(SETPOINT_TOPIC, &GimbalNode::setAttitudeCallback, this);
  this->registerShutdown(SHUTDOWN_TOPIC);
  // clang-format on

  // register loop callback
  this->registerLoopCallback(std::bind(&GimbalNode::loopCallback, this));

  // intialize setpoints
  this->gimbal.setAngle(0.0, 0.0);
  this->configured = true;
  return 0;
}

GimbalNode::~GimbalNode(void) {
  this->gimbal.off();
}

void GimbalNode::setAttitudeCallback(const geometry_msgs::Vector3 &msg) {
  this->set_points << msg.x, msg.y, msg.z;
}

void GimbalNode::trackTargetCallback(const geometry_msgs::Vector3 &msg) {
  Eigen::Vector3d target_cf;
  target_cf << msg.x, msg.y, msg.z;
  this->gimbal.trackTarget(target_cf);
}

int GimbalNode::loopCallback(void) {
   this->gimbal.setAngle(this->set_points(0), this->set_points(1));
   return 0;
 }

}  // end of awesomo namespace

RUN_ROS_NODE(awesomo::GimbalNode, NODE_NAME, NODE_RATE);
