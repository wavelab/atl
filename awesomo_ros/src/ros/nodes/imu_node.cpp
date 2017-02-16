#include "awesomo_ros/nodes/imu_node.hpp"


namespace awesomo {

int IMUNode::configure(std::string node_name, int hz) {
  std::string config_path;

  // imu
  if (this->imu.configure() != 0) {
    log_err("Failed to configure IMU!");
    return -1;
  }

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // register publisher and loop callback
  this->registerPublisher<geometry_msgs::Quaternion>(IMU_TOPIC);
  this->registerLoopCallback(std::bind(&IMUNode::loopCallback, this));

  this->configured = true;
  return 0;
}

int IMUNode::publishData(void) {
  geometry_msgs::Quaternion msg;
  Vec3 euler;
  Quaternion quat;

  if (this->imu.getData() == 0) {
    euler << this->imu.roll, this->imu.pitch, 0.0;
    euler2quat(euler, 321, quat);
    buildMsg(quat, msg);
    this->ros_pubs[IMU_TOPIC].publish(msg);
  }

  return 0;
}

int IMUNode::loopCallback(void) {
  this->publishData();
  return 0;
}

} // eof awesomo namespace

RUN_ROS_NODE(awesomo::IMUNode, NODE_NAME, NODE_RATE);
