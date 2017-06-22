#include "wavesim_ros/nodes/gimbal_node.hpp"

namespace wavesim {
namespace ros {

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

  ros_msg.w = this->frame_orientation.w();
  ros_msg.x = this->frame_orientation.x();
  ros_msg.y = this->frame_orientation.y();
  ros_msg.z = this->frame_orientation.z();

  this->ros_pubs[FRAME_ORIENTATION_RTOPIC].publish(ros_msg);
}

void GimbalNode::jointOrientationCallback(ConstQuaternionPtr &msg) {
  GimbalGClient::jointOrientationCallback(msg);

  // publish joint orientation in ROS
  geometry_msgs::Quaternion ros_msg;

  ros_msg.w = this->joint_orientation.w();
  ros_msg.x = this->joint_orientation.x();
  ros_msg.y = this->joint_orientation.y();
  ros_msg.z = this->joint_orientation.z();

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
  Vec3 euler;
  euler << msg.x, msg.y, msg.z;

  if (this->enable_tracking) {
    GimbalGClient::setAttitude(euler);
  }
}

void GimbalNode::trackTargetCallback(const geometry_msgs::Vector3 msg) {
  Vec3 target_cf;
  target_cf << msg.x, msg.y, msg.z;
  GimbalGClient::trackTarget(target_cf);
}

}  // end of ros namespace
}  // end of wavesim namespace

ROS_NODE_RUN(wavesim::ros::GimbalNode, NODE_NAME, NODE_RATE);
