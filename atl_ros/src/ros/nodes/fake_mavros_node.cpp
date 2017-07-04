#include "atl/ros/nodes/fake_mavros_node.hpp"

namespace atl {

int FakeMavrosNode::configure(std::string node_name, int hz) {
  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // publishers, subscribers and loop callback
  this->registerPublisher<geometry_msgs::PoseStamped>(MAVROS_POSE_TOPIC);
  this->registerPublisher<geometry_msgs::TwistStamped>(MAVROS_VELOCITY_TOPIC);
  this->registerLoopCallback(std::bind(&FakeMavrosNode::loopCallback, this));

  return 0;
}

int FakeMavrosNode::publishPose(void) {
  geometry_msgs::PoseStamped msg;

  msg.pose.position.x = 0.0;
  msg.pose.position.y = 0.0;
  msg.pose.position.z = 0.0;
  msg.pose.orientation.w = 1.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;

  this->ros_pubs[MAVROS_POSE_TOPIC].publish(msg);
  return 0;
}

int FakeMavrosNode::publishVelocity(void) {
  geometry_msgs::TwistStamped msg;

  msg.twist.linear.x = 0.0;
  msg.twist.linear.y = 0.0;
  msg.twist.linear.z = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = 0.0;

  this->ros_pubs[MAVROS_VELOCITY_TOPIC].publish(msg);
  return 0;
}

int FakeMavrosNode::loopCallback(void) {
  this->publishPose();
  this->publishVelocity();
  return 0;
}

}  // namespace atl

RUN_ROS_NODE(atl::FakeMavrosNode, NODE_NAME, NODE_RATE);
