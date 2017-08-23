#include "atl/ros/nodes/imu_node.hpp"

namespace atl {

int IMUNode::configure(const int hz) {
  std::string config_path;

  // ros node
  if (ROSNode::configure(hz) != 0) {
    return -1;
  }
  ROS_GET_PARAM("/quad_frame", this->quad_frame);
  ROS_GET_PARAM("/gimbal_imu", this->gimbal_imu);

  // imu
  if (this->imu.configure() != 0) {
    LOG_ERROR("Failed to configure IMU!");
    return -1;
  }

  // zero imu - assuming it is zero-ed
  LOG_INFO("Zero-ing IMU! DO NOT MOVE THE QUADROTOR!");
  sleep(5);
  for (int i = 0; i < 1000; i++) {
    this->imu.getData();
  }
  this->imu.roll_offset = -1 * this->imu.roll;
  this->imu.pitch_offset = -1 * this->imu.pitch;
  LOG_INFO("Zero-ing complete!");

  // register publisher and subscribers
  // clang-format off
  this->addPublisher<geometry_msgs::Vector3>(IMU_TOPIC);
  if (this->gimbal_imu == "HACK") {
    this->addPublisher<geometry_msgs::Quaternion>(JOINT_ORIENTATION_TOPIC);
  } else if (this->gimbal_imu != "SBGC") {
    LOG_ERROR("Invalid gimbal imu mode [%s]", this->gimbal_imu.c_str());
    return -3;
  }
  // clang-format on

  // loop callback
  this->addLoopCallback(std::bind(&IMUNode::loopCallback, this));

  this->configured = true;
  return 0;
}

int IMUNode::publishIMU(const Vec3 &euler) {
  geometry_msgs::Vector3 msg;
  buildMsg(euler, msg);
  this->ros_pubs[IMU_TOPIC].publish(msg);
  return 0;
}

int IMUNode::publishJointOrientation(const Quaternion &q) {
  geometry_msgs::Quaternion msg;
  buildMsg(q, msg);
  this->ros_pubs[JOINT_ORIENTATION_TOPIC].publish(msg);
  return 0;
}

int IMUNode::loopCallback() {
  Vec3 euler;
  Quaternion q;

  if (this->imu.getData() == 0) {
    euler << -1 * deg2rad(this->imu.roll), deg2rad(this->imu.pitch), 0.0;
    q = euler321ToQuat(euler);
    this->publishIMU(euler);
    if (this->gimbal_imu == "HACK") {
      this->publishJointOrientation(q);
    }
  } else {
    LOG_INFO("Failed to poll IMU for data!");
  }

  return 0;
}

} // eof atl namespace

RUN_ROS_NODE(atl::IMUNode, NODE_RATE);
