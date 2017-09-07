#include "atl/ros/nodes/mission_node.hpp"

namespace atl {

int MissionNode::configure(const int hz) {
  // ROS node
  if (ROSNode::configure(hz) != 0) {
    return -1;
  }

  // Get path to config file
  std::string config_file;
  ROS_GET_PARAM(this->node_name + "/config", config_file);

  // Subscribers
  this->addSubscriber(DJI_RADIO_TOPIC, &MissionNode::radioCallback, this);

  // Clients
  this->addClient<dji_sdk::SDKControlAuthority>(DJI_SDK_SERVICE);
  this->addClient<dji_sdk::MissionWpUpload>(DJI_WAYPOINT_UPLOAD_SERVICE);
  this->addClient<dji_sdk::MissionWpAction>(DJI_WAYPOINT_ACTION_SERVICE);

  // Load mission
  if (this->loadMission(config_file) != 0) {
    LOG_ERROR(EMISSIONLOAD);
    return -1;
  }

  return 0;
}

int MissionNode::sdkControlMode(const bool mode) {
  dji_sdk::SDKControlAuthority msg;
  msg.request.control_enable = mode;

  if (mode) {
    this->ros_clients[DJI_SDK_SERVICE].call(msg);
    if (msg.response.result == false) {
      LOG_ERROR(EDJISDKREQ);
      return -1;
    }
    LOG_INFO(INFO_DJI_SDK_OBTAINED);

  } else {
    this->ros_clients[DJI_SDK_SERVICE].call(msg);
    if (msg.response.result == false) {
      LOG_ERROR(EDJISDKREL);
      return -1;
    }
    LOG_INFO(INFO_DJI_SDK_RELEASED);
  }

  return 0;
}

void MissionNode::radioCallback(const sensor_msgs::Joy &msg) {
  const int mode_switch = msg.axes[4];

  // Arm or disarm SDK mode
  if (mode_switch > 0 && this->state == MISSION_RUNNING) {
    this->state = MISSION_IDEL;
    this->sdkControlMode(false);
    this->stopMission();

  } else if (mode_switch < 0 && this->state == MISSION_IDEL) {
    this->state = MISSION_RUNNING;
    this->sdkControlMode(true);
    this->startMission();
  }
}

int MissionNode::loadMission(const std::string &config_file) {
  ConfigParser parser;
  std::vector<double> waypoint_data;

  // Load config
  parser.addParam("desired_velocity", &this->desired_velocity);
  parser.addParam("waypoints", &waypoint_data);
  if (parser.load(config_file) != 0) {
    return -1;
  }

  // Check number of waypoint data
  if (waypoint_data.size() % 3 != 0 || waypoint_data.size() <= 3) {
    LOG_ERROR(EINVWPDATA);
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

int MissionNode::uploadMission() {
  // Create mission task
  dji_sdk::MissionWaypointTask task;
  task.velocity_range = 10;
  task.idle_velocity = this->desired_velocity;
  task.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
  task.mission_exec_times = 1;
  task.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
  task.trace_mode = dji_sdk::MissionWaypointTask::TRACE_POINT;
  task.action_on_rc_lost = dji_sdk::MissionWaypointTask::ACTION_AUTO;
  task.gimbal_pitch_mode = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;

  for (auto gps_wp : this->gps_waypoints) {
    const double lat = gps_wp(0);
    const double lon = gps_wp(1);
    const double alt = gps_wp(2);

    dji_sdk::MissionWaypoint wp;
    wp.latitude = lat;
    wp.longitude = lon;
    wp.altitude = alt;
    wp.damping_distance = 0;
    wp.target_yaw = 0;
    wp.target_gimbal_pitch = 0;
    wp.turn_mode = 0;
    wp.has_action = 0;
    task.mission_waypoint.push_back(wp);

    LOG_INFO("Waypoint created at [lat:lon:alt]: [%f \t%f \t%f]",
             lat,
             lon,
             alt);
  }

  // Upload mission
  dji_sdk::MissionWpUpload msg;
  msg.request.waypoint_task = task;
  this->ros_clients[DJI_WAYPOINT_UPLOAD_SERVICE].call(msg);

  // Check response
  if (msg.response.result == false) {
    LOG_ERROR(EMISSIONUP);
    return -1;
  }

  return 0;
}

int MissionNode::startMission() {
  // Start mission
  dji_sdk::MissionWpAction wp_action;
  wp_action.request.action = DJI::OSDK::MISSION_ACTION::START;
  this->ros_clients[DJI_WAYPOINT_ACTION_SERVICE].call(wp_action);

  // Check response
  if (wp_action.response.result == false) {
    LOG_ERROR(EMISSIONSTART);
    return -1;
  }

  LOG_INFO("Starting mission!");
  this->state = MISSION_RUNNING;
  return 0;
}

int MissionNode::stopMission() {
  // Start mission
  dji_sdk::MissionWpAction wp_action;
  wp_action.request.action = DJI::OSDK::MISSION_ACTION::STOP;
  this->ros_clients[DJI_WAYPOINT_ACTION_SERVICE].call(wp_action);

  // Check response
  if (wp_action.response.result == false) {
    LOG_ERROR(EMISSIONSTOP);
    return -1;
  }

  LOG_INFO("Stopping mission!");
  if (this->state != MISSION_COMPLETED) {
    this->state = MISSION_IDEL;
  }
  return 0;
}

} // namespace atl

RUN_ROS_NODE(atl::MissionNode, NODE_RATE);
