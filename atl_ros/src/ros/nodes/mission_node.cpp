#include "atl/ros/nodes/mission_node.hpp"

namespace atl {

int MissionNode::configure(const std::string &node_name, int hz) {
  std::string config_file;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // configure mission
  ROS_GET_PARAM("/mission/config", config_file);
  if (this->mission.configure(config_file) != 0) {
    ROS_ERROR("Failed to configure mission!");
    return -2;
  }

  // configure dji topics
  // clang-format off
  this->registerSubscriber(DJI_GPS_POSITION_TOPIC, &MissionNode::gpsPositionCallback, this);
  this->registerSubscriber(DJI_LOCAL_POSITION_TOPIC, &MissionNode::localPositionCallback, this);
  this->registerSubscriber(DJI_ATTITUDE_TOPIC, &MissionNode::attitudeCallback, this);
  this->registerSubscriber(DJI_VELOCITY_TOPIC, &MissionNode::velocityCallback, this);
  this->registerSubscriber(DJI_RADIO_TOPIC, &MissionNode::radioCallback, this);
  // clang-format on

  // dji
  this->dji = new DJIDrone(*this->ros_nh);

  this->configured = true;
  return 0;
}

void MissionNode::gpsPositionCallback(const dji_sdk::GlobalPosition &msg) {
  this->latitude = msg.latitude;
  this->longitude = msg.longitude;
}

void MissionNode::localPositionCallback(const dji_sdk::LocalPosition &msg) {
  Vec3 pos_ned, pos_enu;

  pos_ned(0) = msg.x;
  pos_ned(1) = msg.y;
  pos_ned(2) = msg.z;
  ned2enu(pos_ned, pos_enu);

  this->quadrotor.pose.position = pos_enu;
}

void MissionNode::attitudeCallback(const dji_sdk::AttitudeQuaternion &msg) {
  Quaternion orientation_ned, orientation_nwu;

  orientation_ned.w() = msg.q0;
  orientation_ned.x() = msg.q1;
  orientation_ned.y() = msg.q2;
  orientation_ned.z() = msg.q3;

  // transform pose position and orientation
  // from NED to NWU
  ned2nwu(orientation_ned, orientation_nwu);

  this->quadrotor.pose.orientation = orientation_nwu;
}

void MissionNode::velocityCallback(const dji_sdk::Velocity &msg) {
  Vec3 vel_ned, vel_enu;

  // convert DJI msg to Eigen vector
  vel_ned(0) = msg.vx;
  vel_ned(1) = msg.vy;
  vel_ned(2) = msg.vz;

  // transform velocity in NED to ENU
  ned2enu(vel_ned, vel_enu);

  // update
  this->quadrotor.setVelocity(vel_enu);
}

void MissionNode::radioCallback(const dji_sdk::RCChannels &msg) {
  if (msg.mode > 0 && this->offboard == true) {
    this->offboard = false;
    this->offboardModeOff();
    this->dji->mission_cancel();

  } else if (msg.mode < 0 && this->offboard == false) {
    this->offboard = true;
    if (this->offboardModeOn() == 0) {
      this->executeMission();
    }
  }
}

int MissionNode::takeoff() {
  // pre-check
  if (this->configured == false) {
    return -1;
  } else
    (this->offboard == false) {
      return -2;
    }

  // takeoff

  return 0;
}

int MissionNode::land() {
  // pre-check
  if (this->configured == false) {
    return -1;
  } else
    (this->offboard == false) {
      return -2;
    }

  // land

  return 0;
}

int MissionNode::offboardModeOn() {
  if (this->dji->request_sdk_permission_control() != true) {
    LOG_ERROR("Failed to release DJI SDK control!");
    return -1;
  }
  LOG_INFO("Obtained DJI SDK control!");

  return 0;
}

int MissionNode::offboardModeOff() {
  if (this->dji->release_sdk_permission_control() != true) {
    LOG_ERROR("Failed to release DJI SDK control!");
    return -1;
  }
  LOG_INFO("Released DJI SDK control!");

  return 0;
}

int MissionNode::executeMission() {
  LOG_INFO("Executing Mission!");

  // execute mission
  this->buildMission();
  // this->dji->mission_start();

  return 0;
}

}  // namespace atl

RUN_ROS_NODE(atl::MissionNode, NODE_NAME, NODE_RATE);
