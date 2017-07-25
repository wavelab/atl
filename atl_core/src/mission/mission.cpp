#include "atl/mission/mission.hpp"

namespace atl {

int Mission::configure(const std::string &config_file) {
  ConfigParser parser;
  std::vector<double> waypoint_data;

  // load config
  parser.addParam("velocity", &this->velocity);
  parser.addParam("waypoints", &waypoint_data);
  if (parser.load(config_file) != 0) {
    return -1;
  }

  // check number waypoint data
  if (waypoint_data.size() % 5 != 0) {
    LOG_ERROR("Invalid number of waypoint data!");
    return -1;
  }

  // convert waypoint data into waypoints
  for (size_t i = 0; i < waypoint_data.size(); i += 5) {
    double lat = waypoint_data[i];
    double lon = waypoint_data[i + 1];
    double alt = waypoint_data[i + 2];
    double staytime = waypoint_data[i + 3];
    double heading = waypoint_data[i + 4];

    // check lat, lon
    if (fltcmp(lat, 0.0) == 0.0 || fltcmp(lon, 0.0) == 0.0) {
      LOG_ERROR(EINVLATLON, lat, lon);
      return -1;
    }

    // check alt
    if (fltcmp(alt, 0.0) == 0.0) {
      LOG_ERROR(EINVALT, alt);
      return -1;
    }

    // check staytime
    if (staytime < 0.0) {
      LOG_ERROR(EINVSTAY, staytime);
      return -1;
    }

    // check heading
    if (heading > deg2rad(180.0) || heading < deg2rad(-180.0)) {
      LOG_ERROR(EINVHEADING, heading);
      return -1;
    }

    this->waypoints.emplace_back(lat, lon, alt, staytime, heading);
  }

  // check waypoints
  if (this->check_waypoints && this->checkWaypoints() != 0) {
    return -2;
  }

  this->configured = true;
  return 0;
}

int Mission::checkWaypoints() {
  // pre-check
  if (this->waypoints.size() == 0) {
    return -1;
  }

  // check waypoints
  Waypoint last_wp = this->waypoints.front();
  for (size_t i = 1; i < this->waypoints.size(); i++) {
    // calculate distance between current and last waypoint
    Waypoint wp = this->waypoints[i];

    // check distance
    if (last_wp.distance(wp) > 20.0) {
      LOG_ERROR(EDISTLATLON,
                (int) i + 1,
                wp.latitude,
                wp.longitude,
                this->waypoint_threshold);
      return -2;
    }

    // update last waypoint
    last_wp = wp;
  }

  return 0;
}

}  // namespace atl
