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

dji_sdk::WaypointList MissionNode::buildMission() {
  dji_sdk::WaypointList waypoint_list;

  // create waypoints list
  for (auto wp : this->mission.waypoints) {
    dji_sdk::Waypoint waypoint;
    waypoint.latitude = wp.latitude;
    waypoint.longitude = wp.longitude;
    waypoint.altitude = wp.altitude;
    waypoint.staytime = wp.staytime;
    waypoint.heading = wp.heading;

    waypoint_list.waypoint_list.push_back(waypoint);
  }

  // set mission velocity
  this->dji->mission_waypoint_set_speed(this->mission.velocity);
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
  dji_sdk::WaypointList waypoint_list = this->buildMission();
  this->dji->waypoint_navigation_send_request(waypoint_list);

  return 0;
}

}  // namespace atl

RUN_ROS_NODE(atl::MissionNode, NODE_NAME, NODE_RATE);
