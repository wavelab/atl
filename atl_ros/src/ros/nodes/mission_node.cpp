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
  this->registerSubscriber(DJI_RADIO_TOPIC, &MissionNode::radioCallback, this);
  // clang-format on

  // dji
  this->dji = new DJIDrone(*this->ros_nh);

  this->configured = true;
  return 0;
}

dji_sdk::MissionWaypointTask MissionNode::buildMission() {
  dji_sdk::MissionWaypointTask mission_task;

  // mission general settings
  mission_task.velocity_range = this->mission.velocity;
  mission_task.idle_velocity = 0;
  mission_task.action_on_finish = 0;
  mission_task.mission_exec_times = 1;
  mission_task.yaw_mode = 4;
  mission_task.trace_mode = 0;
  mission_task.action_on_rc_lost = 0;
  mission_task.gimbal_pitch_mode = 0;

  // create waypoints list
  for (auto wp : this->mission.waypoints) {
    dji_sdk::MissionWaypoint waypoint;

    waypoint.latitude = wp.latitude;
    waypoint.longitude = wp.longitude;
    waypoint.altitude = wp.altitude;  // relative to takeoff point, not sea level
    waypoint.damping_distance = 0.1;
    waypoint.target_yaw = wp.heading;
    waypoint.target_gimbal_pitch = 0;
    waypoint.turn_mode = 0;
    waypoint.has_action = 0;

    mission_task.mission_waypoint.push_back(waypoint);
  }

  return mission_task;
}

void MissionNode::radioCallback(const dji_sdk::RCChannels &msg) {
  if (msg.mode > 0 && this->offboard == true) {
    this->offboard = false;
    this->offboardModeOff();
    this->dji->mission_cancel();

  } else if (msg.mode < 0 && this->offboard == false) {
    this->offboard = true;
    if (this->offboardModeOn() == true) {
      this->executeMission();
    }
  }
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
  dji_sdk::MissionWaypointTask mission_task = this->buildMission();
  this->dji->mission_waypoint_upload(mission_task);
  this->dji->mission_start();

  return 0;
}

}  // namespace atl

RUN_ROS_NODE(atl::MissionNode, NODE_NAME, NODE_RATE);
