#include "atl/ros/nodes/gimbal_node.hpp"

namespace atl {

int GimbalNode::configure(std::string node_name, int hz) {
  std::string config_file;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // gimbal
  ROS_GET_PARAM("/gimbal_config", config_file);
  ROS_GET_PARAM("/gimbal_imu", this->gimbal_imu);
  if (this->gimbal.configure(config_file) != 0) {
    ROS_ERROR("Failed to configure Gimbal!");
    return -2;
  };

  // register publisher and subscribers
  // clang-format off
  this->registerPublisher<geometry_msgs::Vector3>(SBGC_IMU_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(SBGC_RAW_ENCODER_TOPIC);
  this->registerPublisher<geometry_msgs::Vector3>(POSITION_TOPIC);
  this->registerPublisher<geometry_msgs::Quaternion>(FRAME_ORIENTATION_TOPIC);
  this->registerPublisher<geometry_msgs::Quaternion>(ENCODER_ORIENTATION_TOPIC);

  if (this->gimbal_imu == "SBGC") {
    this->registerPublisher<geometry_msgs::Quaternion>(JOINT_ORIENTATION_TOPIC);
  } else if (this->gimbal_imu != "HACK") {
    log_err("Invalid gimbal imu mode [%s]", this->gimbal_imu.c_str());
    return -3;
  }

  this->registerSubscriber(QUAD_POSE_TOPIC, &GimbalNode::quadPoseCallback, this);
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

int GimbalNode::publishIMU(Vec3 euler) {
  geometry_msgs::Vector3 msg;
  buildMsg(euler, msg);
  this->ros_pubs[SBGC_IMU_TOPIC].publish(msg);
  return 0;
}

int GimbalNode::publishRawEncoder(Vec3 encoder_euler){
  geometry_msgs::Vector3 msg;
  buildMsg(encoder_euler, msg);
  this->ros_pubs[SBGC_RAW_ENCODER_TOPIC].publish(msg);
  return 0;
}

int GimbalNode::publishPosition(Vec3 pos) {
  geometry_msgs::Vector3 msg;
  buildMsg(pos, msg);
  this->ros_pubs[POSITION_TOPIC].publish(msg);
  return 0;
}

int GimbalNode::publishFrameOrientation(Quaternion q) {
  geometry_msgs::Quaternion msg;
  buildMsg(q, msg);
  this->ros_pubs[FRAME_ORIENTATION_TOPIC].publish(msg);
  return 0;
}

int GimbalNode::publishJointOrientation(Quaternion q) {
  geometry_msgs::Quaternion msg;
  buildMsg(q, msg);
  this->ros_pubs[JOINT_ORIENTATION_TOPIC].publish(msg);
  return 0;
}

int GimbalNode::publishEncoderOrientation(Quaternion q) {
  geometry_msgs::Quaternion msg;
  buildMsg(q, msg);
  this->ros_pubs[ENCODER_ORIENTATION_TOPIC].publish(msg);
  return 0;
}

void GimbalNode::quadPoseCallback(const geometry_msgs::PoseStamped &msg) {
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

void GimbalNode::setAttitudeCallback(const geometry_msgs::Vector3 &msg) {
  this->set_points << msg.x, msg.y, msg.z;
}

void GimbalNode::trackTargetCallback(const geometry_msgs::Vector3 &msg) {
  Eigen::Vector3d target_cf;
  target_cf << msg.x, msg.y, msg.z;
  this->gimbal.trackTarget(target_cf);
}

int GimbalNode::loopCallback(void) {
  Vec3 euler;
  Quaternion q;
  Vec3 encoder_euler;
  Quaternion encoder_q;

  // set gimbal attitude
  this->gimbal.setAngle(this->set_points(0), this->set_points(1));

  // publish gimbal joint orientation
  if (this->gimbal_imu == "SBGC" && this->gimbal.updateGimbalStates() == 0) {
    // imu reading
    euler(0) = this->gimbal.camera_angles(0);
    euler(1) = this->gimbal.camera_angles(1);
    euler(2) = 0;
    euler2quat(euler, 321, q);
    this->publishIMU(euler);
    this->publishJointOrientation(q);

    // encoder reading
    encoder_euler(0) = this->gimbal.encoder_angles(0);
    encoder_euler(1) = this->gimbal.encoder_angles(1);
    encoder_euler(2) = 0.0; // There is no yaw encoder (yet)
    euler2quat(encoder_euler, 321, encoder_q);

    // publish imu and encoder readings
    this->publishRawEncoder(encoder_euler);
    this->publishEncoderOrientation(encoder_q);
  }

  return 0;
}

}  // end of atl namespace

RUN_ROS_NODE(atl::GimbalNode, NODE_NAME, NODE_RATE);
