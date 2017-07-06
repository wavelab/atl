#include "atl/gazebo/bridge/gimbal_node.hpp"

namespace atl {
namespace gazebo_bridge {

int GimbalNode::configure(const std::string &node_name, int hz) {
  this->quad_frame = "NWU";
  this->enable_tracking = true;

  // setup ros node
  // clang-format off
  ROSNode::configure(node_name, hz);
  ROSNode::registerPublisher<geometry_msgs::Quaternion>(FRAME_ORIENTATION_RTOPIC);
  ROSNode::registerPublisher<geometry_msgs::Quaternion>(JOINT_ORIENTATION_RTOPIC);
  ROSNode::registerPublisher<geometry_msgs::Vector3>(POSITION_RTOPIC);
  ROSNode::registerSubscriber(POSE_RTOPIC, &GimbalNode::quadPoseCallback, this);
  ROSNode::registerSubscriber(SETPOINT_RTOPIC, &GimbalNode::setAttitudeCallback, this);
  ROSNode::registerSubscriber(TRACK_RTOPIC, &GimbalNode::trackTargetCallback, this);
  // clang-format on

  // connect gazebo client
  if (GimbalGClient::configure() != 0) {
    ROS_ERROR("Failed to configure GimbalGClient!");
    return -1;
  }

  return 0;
}

void GimbalNode::frameOrientationCallback(ConstQuaternionPtr &msg) {
  GimbalGClient::frameOrientationCallback(msg);

  // publish frame orientation in ROS
  geometry_msgs::Quaternion ros_msg;
  buildMsg(this->frame_orientation, ros_msg);
  this->ros_pubs[FRAME_ORIENTATION_RTOPIC].publish(ros_msg);
}

void GimbalNode::jointOrientationCallback(ConstQuaternionPtr &msg) {
  GimbalGClient::jointOrientationCallback(msg);

  geometry_msgs::Quaternion ros_msg;
  buildMsg(this->joint_orientation, ros_msg);
  this->ros_pubs[JOINT_ORIENTATION_RTOPIC].publish(ros_msg);
}

void GimbalNode::quadPoseCallback(const geometry_msgs::PoseStamped &msg) {
  geometry_msgs::Vector3 ros_msg;

  ros_msg.x = msg.pose.position.x;
  ros_msg.y = msg.pose.position.y;
  ros_msg.z = msg.pose.position.z;

  this->ros_pubs[POSITION_RTOPIC].publish(ros_msg);
}

void GimbalNode::setAttitudeCallback(const geometry_msgs::Vector3 msg) {
  if (this->enable_tracking) {
    Vec3 euler{msg.x, msg.y, msg.z};
    GimbalGClient::setAttitude(euler);
  }
}

void GimbalNode::trackTargetCallback(const geometry_msgs::Vector3 msg) {
  Vec3 target_cf{msg.x, msg.y, msg.z};
  GimbalGClient::trackTarget(target_cf);
}

}  // namespace gazebo_bridge
}  // namespace atl

RUN_ROS_NODE(atl::gazebo_bridge::GimbalNode, NODE_NAME, NODE_RATE);
