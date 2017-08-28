#include "atl/ros/nodes/gimbal_node.hpp"

namespace atl {

int GimbalNode::configure(const int hz) {
  std::string config_file;

  // ros node
  if (ROSNode::configure(hz) != 0) {
    return -1;
  }

  // gimbal
  ROS_GET_PARAM(this->node_name + "/config", config_file);
  if (this->gimbal.configure(config_file) != 0) {
    ROS_ERROR("Failed to configure Gimbal!");
    return -2;
  };

  // register publisher and subscribers
  // clang-format off
  this->addPublisher<geometry_msgs::Vector3>(SBGC_IMU_TOPIC);
  this->addPublisher<geometry_msgs::Vector3>(SBGC_RAW_ENCODER_TOPIC);
  this->addPublisher<geometry_msgs::Vector3>(POSITION_TOPIC);
  this->addPublisher<geometry_msgs::Quaternion>(FRAME_ORIENTATION_TOPIC);
  this->addPublisher<geometry_msgs::Quaternion>(ENCODER_ORIENTATION_TOPIC);
  this->addPublisher<geometry_msgs::Quaternion>(JOINT_ORIENTATION_TOPIC);

  this->addSubscriber(ACTIVATE_TOPIC, &GimbalNode::activateCallback, this);
  this->addSubscriber(QUAD_POSE_TOPIC, &GimbalNode::quadPoseCallback, this);
  this->addSubscriber(TRACK_TOPIC, &GimbalNode::trackTargetCallback, this);
  this->addSubscriber(SETPOINT_TOPIC, &GimbalNode::setAttitudeCallback, this);
  this->addShutdownListener(SHUTDOWN_TOPIC);
  // clang-format on

  // register loop callback
  this->addLoopCallback(std::bind(&GimbalNode::loopCallback, this));

  // intialize setpoints
  this->gimbal.setAngle(0.0, 0.0);
  this->configured = true;
  return 0;
}

GimbalNode::~GimbalNode() { this->gimbal.off(); }

int GimbalNode::publishIMU(const Vec3 &euler) {
  geometry_msgs::Vector3 msg;
  buildMsg(euler, msg);
  this->ros_pubs[SBGC_IMU_TOPIC].publish(msg);
  return 0;
}

int GimbalNode::publishRawEncoder(const Vec3 &encoder_rpy) {
  geometry_msgs::Vector3 msg;
  buildMsg(encoder_rpy, msg);
  this->ros_pubs[SBGC_RAW_ENCODER_TOPIC].publish(msg);
  return 0;
}

int GimbalNode::publishPosition(const Vec3 &pos) {
  geometry_msgs::Vector3 msg;
  buildMsg(pos, msg);
  this->ros_pubs[POSITION_TOPIC].publish(msg);
  return 0;
}

int GimbalNode::publishFrameOrientation(const Quaternion &q) {
  geometry_msgs::Quaternion msg;
  buildMsg(q, msg);
  this->ros_pubs[FRAME_ORIENTATION_TOPIC].publish(msg);
  return 0;
}

int GimbalNode::publishJointOrientation(const Quaternion &q) {
  geometry_msgs::Quaternion msg;
  buildMsg(q, msg);
  this->ros_pubs[JOINT_ORIENTATION_TOPIC].publish(msg);
  return 0;
}

int GimbalNode::publishEncoderOrientation(const Quaternion &q) {
  geometry_msgs::Quaternion msg;
  buildMsg(q, msg);
  this->ros_pubs[ENCODER_ORIENTATION_TOPIC].publish(msg);
  return 0;
}

void GimbalNode::activateCallback(const std_msgs::Bool &msg) {
  bool activate;
  convertMsg(msg, activate);

  if (activate) {
    this->gimbal.on();
  } else {
    this->gimbal.off();
  }
}

void GimbalNode::quadPoseCallback(const geometry_msgs::PoseStamped &msg) {
  const Vec3 pos{msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
  const Quaternion q{msg.pose.orientation.w,
                     msg.pose.orientation.x,
                     msg.pose.orientation.y,
                     msg.pose.orientation.z};

  this->publishPosition(pos);
  this->publishFrameOrientation(q);
}

void GimbalNode::setAttitudeCallback(const geometry_msgs::Vector3 &msg) {
  this->set_points << msg.x, msg.y, msg.z;
}

void GimbalNode::trackTargetCallback(const geometry_msgs::Vector3 &msg) {
  const Vec3 target_cf{msg.x, msg.y, msg.z};
  this->gimbal.trackTarget(target_cf);
}

int GimbalNode::loopCallback() {
  // set gimbal attitude
  this->gimbal.setAngle(this->set_points(0), this->set_points(1));

  // obtain gimbal states
  if (this->gimbal.updateGimbalStates() == 0) {
    // imu reading
    this->imu_rpy << this->gimbal.camera_angles(0),
        this->gimbal.camera_angles(1), 0.0;

    // encoder reading
    this->encoder_rpy << this->gimbal.encoder_angles(0),
        this->gimbal.encoder_angles(1),
        0.0; // There is no yaw encoder (yet)
  }

  // publish imu readings
  const Quaternion imu_quat = euler321ToQuat(this->imu_rpy);
  this->publishIMU(this->imu_rpy);
  this->publishJointOrientation(imu_quat);

  // publish encoder readings
  const Quaternion encoder_q = euler321ToQuat(encoder_rpy);
  this->publishRawEncoder(encoder_rpy);
  this->publishEncoderOrientation(encoder_q);

  // debug
  // std::cout << "IMU RPY: " << rad2deg(this->imu_rpy(0));
  // std::cout << " " << rad2deg(this->imu_rpy(1));
  // std::cout << " " << rad2deg(this->imu_rpy(2)) << std::endl;

  // std::cout << "ENCODER RPY: " << rad2deg(this->encoder_rpy(0));
  // std::cout << " " << rad2deg(this->encoder_rpy(1));
  // std::cout << " " << rad2deg(this->encoder_rpy(2)) << std::endl;
  // std::cout << std::endl;

  return 0;
}

} // namespace atl

RUN_ROS_NODE(atl::GimbalNode, NODE_RATE);
