#include "awesomo_ros/nodes/imu_node.hpp"


namespace awesomo {

int IMUNode::configure(std::string node_name, int hz) {
  std::string config_path;

  // imu
  if (this->imu.configure() != 0) {
    log_err("Failed to configure IMU!");
    return -1;
  }

  // zero imu - assuming it is zero-ed
  log_info("Zero-ing IMU! DO NOT MOVE THE QUADROTOR!");
  sleep(5);
  for (int i = 0; i < 1000; i++) {
    this->imu.getData();
  }
  this->imu.roll_offset = -1 * this->imu.roll;
  this->imu.pitch_offset = -1 * this->imu.pitch;
  log_info("Zero-ing complete!");

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }
  ROS_GET_PARAM("/quad_frame", this->quad_frame);

  // register publisher and subscribers
  // clang-format off
  this->registerPublisher<geometry_msgs::Vector3>(IMU_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(POSITION_TOPIC);
  this->registerPublisher<geometry_msgs::Quaternion>(FRAME_ORIENTATION_TOPIC);
  this->registerPublisher<geometry_msgs::Quaternion>(JOINT_ORIENTATION_TOPIC);
  this->registerSubscriber(QUAD_POSE_TOPIC, &IMUNode::quadPoseCallback, this);
  // clang-format on

  // loop callback
  this->registerLoopCallback(std::bind(&IMUNode::loopCallback, this));

  this->configured = true;
  return 0;
}

int IMUNode::publishIMU(Vec3 euler) {
  geometry_msgs::Vector3 msg;
  buildMsg(euler, msg);
  this->ros_pubs[IMU_TOPIC].publish(msg);
  return 0;
}

int IMUNode::publishPosition(Vec3 pos) {
  geometry_msgs::Vector3 msg;
  buildMsg(pos, msg);
  this->ros_pubs[POSITION_TOPIC].publish(msg);
  return 0;
}

int IMUNode::publishFrameOrientation(Quaternion q) {
  geometry_msgs::Quaternion msg;
  buildMsg(q, msg);
  this->ros_pubs[FRAME_ORIENTATION_TOPIC].publish(msg);
  return 0;
}

int IMUNode::publishJointOrientation(Quaternion q) {
  geometry_msgs::Quaternion msg;
  buildMsg(q, msg);
  this->ros_pubs[JOINT_ORIENTATION_TOPIC].publish(msg);
  return 0;
}

void IMUNode::quadPoseCallback(const geometry_msgs::PoseStamped &msg) {
  Vec3 pos;
  Quaternion q;

  pos(0)  = msg.pose.position.x;
  pos(1)  = msg.pose.position.y;
  pos(2)  = msg.pose.position.z;

  q.w() = msg.pose.orientation.w;
  q.x() = msg.pose.orientation.x;
  q.y() = msg.pose.orientation.y;
  q.z() = msg.pose.orientation.z;

  this->publishPosition(pos);
  this->publishFrameOrientation(q);
}

int IMUNode::loopCallback(void) {
  Vec3 euler;
  Quaternion q;

  if (this->imu.getData() == 0) {
    euler << -1 * deg2rad(this->imu.roll), deg2rad(this->imu.pitch), 0.0;
    euler2quat(euler, 321, q);
    this->publishIMU(euler);
    this->publishJointOrientation(q);
  } else {
    log_info("Failed to poll IMU for data!");
  }

  return 0;
}

} // eof awesomo namespace

RUN_ROS_NODE(awesomo::IMUNode, NODE_NAME, NODE_RATE);
