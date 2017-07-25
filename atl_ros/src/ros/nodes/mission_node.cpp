#include "atl/ros/nodes/mission_node.hpp"

namespace atl {

int MissionNode::configure(const std::string &node_name, int hz) {
  std::string config_path;

  // ros node
  if (ROSNode::configure(node_name, hz) != 0) {
    return -1;
  }

  // configure dji topics
  this->registerSubscriber(
    DJI_RADIO_TOPIC, &MissionNode::djiRadioCallback, this);

  // dji
  this->dji = new DJIDrone(*this->ros_nh);

  this->configured = true;
  return 0;
}

void MissionNode::djiRadioCallback(const dji_sdk::RCChannels &msg) {
  if (msg.mode > 0 && this->offboard == true) {
    this->offboard = false;
    this->djiOffboardModeOff();

  } else if (msg.mode < 0 && this->offboard == false) {
    this->offboard = true;
    this->djiOffboardModeOn();
  }
}

int MissionNode::djiOffboardModeOn() {
  if (this->dji->request_sdk_permission_control() != true) {
    LOG_ERROR("Failed to release DJI SDK control!");
    return -1;
  }
  LOG_INFO("Obtained DJI SDK control!");

  return 0;
}

int MissionNode::djiOffboardModeOff() {
  if (this->dji->release_sdk_permission_control() != true) {
    LOG_ERROR("Failed to release DJI SDK control!");
    return -1;
  }
  LOG_INFO("Released DJI SDK control!");

  return 0;
}

int MissionNode::runMission() {
  dji_sdk::WaypointList waypoint_list;

  for (int i = 0; i < 10; i++) {
    dji_sdk::Waypoint waypoint;
    waypoint.latitude = 0.0;
    waypoint.longitude = 0.0;
    waypoint.altitude = 0.0;
    waypoint.staytime = 0;
    waypoint.heading = 0;

    // waypoint_list.push_back(waypoint);
  }
  this->dji->mission_waypoint_set_speed(1.0);
  this->dji->waypoint_navigation_send_request(waypoint_list);

  return 0;
}

}  // namespace atl

RUN_ROS_NODE(atl::MissionNode, NODE_NAME, NODE_RATE);
