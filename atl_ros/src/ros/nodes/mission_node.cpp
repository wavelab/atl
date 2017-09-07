#include "atl/ros/nodes/mission_node.hpp"

namespace atl {

int MissionNode::configure(const int hz) {
  // ROS node
  if (ROSNode::configure(hz) != 0) {
    return -1;
  }

  return 0;
}

int MissionNode::loadMission() {
  ConfigParser parser;
  std::vector<double> waypoint_data;

  // Load config
  parser.addParam("desired_velocity", &this->desired_velocity);
  parser.addParam("waypoints", &waypoint_data);
  if (parser.load(config_file) != 0) {
    return -1;
  }

  // Check number of waypoint data
  if (waypoint_data.size() % 3 != 0 || waypoint_data <= 3) {
    LOG_ERROR("Invalid number of waypoint data!");
    return -1;
  }

  // Load first GPS waypoint
  const double lat = waypoint_data[0];
  const double lon = waypoint_data[1];
  const double alt = waypoint_data[2];
  this->gps_waypoints.emplace_back(lat, lon, alt);

  // Load GPS waypoints
  for (size_t i = 1; i < waypoint_data.size(); i += 3) {
    const double lat = waypoint_data[i];
    const double lon = waypoint_data[i + 1];
    const double alt = waypoint_data[i + 2];
    const double last_lat = waypoint_data[(i - 1)];
    const double last_lon = waypoint_data[(i - 1) + 1];
    const double last_alt = waypoint_data[(i - 1) + 2];

    // Check lat, lon
    if (fltcmp(lat, 0.0) == 0.0 || fltcmp(lon, 0.0) == 0.0) {
      LOG_ERROR(EINVLATLON, lat, lon);
      return -1;
    }

    // Check alt
    if (fltcmp(alt, 0.0) == 0.0) {
      LOG_ERROR(EINVALT, alt);
      return -1;
    }

    // Check distance between waypoints
    double dist = latlon_dist(last_lat, last_lon, lat, lon);
    if (dist > this->threshold_waypoint_gap) {
      LOG_ERROR(EDISTLATLON,
                (int) i + 1,
                lat,
                lon,
                this->threshold_waypoint_gap);
      return -2;
    }

    this->gps_waypoints.emplace_back(lat, lon, alt);
  }

  return 0;
}

int MissionNode::uploadMission(dji_sdk::MissionWaypointTask& task) {
//   for (auto wp : wp_list) {
//     ROS_INFO("Waypoint created at [lat:lon:alt]: [%f \t%f \t%f]\n ",
//              wp.latitude,
//              wp.longitude,
//              wp.altitude);
//
//     dji_sdk::MissionWaypoint mission_waypoint;
//     mission_waypoint.latitude= wp->latitude;
//     mission_waypoint.longitude = wp->longitude;
//     mission_waypoint.altitude = wp->altitude;
//     mission_waypoint.damping_distance = 0;
//     mission_waypoint.target_yaw = 0;
//     mission_waypoint.target_gimbal_pitch = 0;
//     mission_waypoint.turn_mode = 0;
//     mission_waypoint.has_action = 0;
//     waypointTask.mission_waypoint.push_back(mission_waypoint);
//   }

  // Upload mission
  dji_sdk::MissionWpUpload msg;
  msg.request.waypoint_task = task;
  waypoint_upload_service.call(msg);

  // Check response
  if (msg.response.result == false) {
    ROS_ERROR("Failed to upload mission!\n");
    return -1;
  }

  return 0;
}

bool MissionNode::createMission() {
  // Initialize waypoint task
  dji_sdk::MissionWaypointTask wp_task;

  wp_task.damping = 0;
  wp_task.yaw = 0;
  wp_task.gimbalPitch = 0;
  wp_task.turnMode = 0;
  wp_task.hasAction = 0;
  wp_task.actionTimeLimit = 100;
  wp_task.actionNumber = 0;
  wp_task.actionRepeat = 0;

  for (int i = 0; i < 16; i++) {
    wp.commandList[i] = 0;
    wp.commandParameter[i] = 0;
  }

  // Create Waypoints
  // const float64_t increment = 0.000001 / 180;
  // const float32_t start_alt = 10;
  // ROS_INFO("Creating Waypoints..\n");
  // std::vector<WayPointSettings> generatedWaypts =
  //   createWaypoints(numWaypoints, increment, start_alt);

  // Upload waypoints
  ROS_INFO("Uploading Waypoints..\n");
  uploadWaypoints(generatedWaypts, responseTimeout, waypointTask);

  // Init mission
  // ROS_INFO("Initializing Waypoint Mission..\n");
  // if (initWaypointMission(waypointTask).result) {
  //   ROS_INFO("Waypoint upload command sent successfully");
  // } else {
  //   ROS_WARN("Failed sending waypoint upload command");
  //   return false;
  // }
  //
  // // Waypoint Mission: Start
  // if (missionAction(DJI_MISSION_TYPE::WAYPOINT,
  //                   MISSION_ACTION::START)
  //       .result)
  // {
  //   ROS_INFO("Mission start command sent successfully");
  // } else {
  //   ROS_WARN("Failed sending mission start command");
  //   return false;
  // }

  return true;
}

} // eof atl namespace

RUN_ROS_NODE(atl::IMUNode, NODE_RATE);
