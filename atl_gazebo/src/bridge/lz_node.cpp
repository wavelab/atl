#include "atl/gazebo/bridge/lz_node.hpp"

namespace atl {
namespace gazebo_bridge {

int LZNode::configure(int hz) {
  // setup ros node
  // clang-format off
  ROSNode::configure(hz);
  ROSNode::addPublisher<geometry_msgs::Pose>(POSE_RTOPIC);
  ROSNode::addSubscriber(POSITION_SET_RTOPIC, &LZNode::positionCallback, this);
  ROSNode::addSubscriber(VELOCITY_SET_RTOPIC, &LZNode::velocityCallback, this);
  ROSNode::addSubscriber(ANGULAR_VEL_SET_RTOPIC, &LZNode::angularVelocityCallback, this);
  // clang-format on

  // connect gazebo client
  if (LZGClient::configure() != 0) {
    ROS_ERROR("Failed to configure LZGClient!");
    return -1;
  }

  return 0;
}

void LZNode::poseCallback(ConstPosePtr &msg) {
  LZGClient::poseCallback(msg);

  // build ros msg and publish
  geometry_msgs::Pose ros_msg;

  // convert NWU to ENU
  ros_msg.position.x = -this->pose.Pos().Y();
  ros_msg.position.y = this->pose.Pos().X();
  ros_msg.position.z = this->pose.Pos().Z();

  ros_msg.orientation.w = this->pose.Rot().W();
  ros_msg.orientation.x = this->pose.Rot().X();
  ros_msg.orientation.y = this->pose.Rot().Y();
  ros_msg.orientation.z = this->pose.Rot().Z();

  this->ros_pubs[POSE_RTOPIC].publish(ros_msg);
}

void LZNode::positionCallback(const geometry_msgs::Vector3 &msg) {
  Vec2 nwu;

  // transform from ENU to NWU
  nwu(0) = msg.y;
  nwu(1) = -msg.x;

  this->setXYPosition(nwu(0), nwu(1));
}

void LZNode::velocityCallback(const std_msgs::Float64 &msg) {
  this->setVelocity(msg.data);
}

void LZNode::angularVelocityCallback(const std_msgs::Float64 &msg) {
  this->setAngularVelocity(msg.data);
}

} // namespace gazebo_bridge
} // namespace atl

RUN_ROS_NODE(atl::gazebo_bridge::LZNode, NODE_RATE);
