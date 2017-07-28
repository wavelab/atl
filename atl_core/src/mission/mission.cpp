#include "atl/mission/mission.hpp"

namespace atl {

int Mission::configure(const std::string &config_file,
                       double home_lat,
                       double home_lon) {
  ConfigParser parser;
  std::vector<double> waypoint_data;

  // load config
  // clang-format off
  parser.addParam("desired_velocity", &this->desired_velocity);
  parser.addParam("look_ahead_dist", &this->look_ahead_dist);
  parser.addParam("threshold_waypoint_gap", &this->threshold_waypoint_gap);
  parser.addParam("threshold_waypoint_reached", &this->threshold_waypoint_reached);
  parser.addParam("waypoints", &waypoint_data);
  // clang-format on
  if (parser.load(config_file) != 0) {
    return -1;
  }

  // check number waypoint data
  if (waypoint_data.size() % 3 != 0) {
    LOG_ERROR("Invalid number of waypoint data!");
    return -1;
  }

  // convert waypoint data into waypoints in the local frame
  for (size_t i = 0; i < waypoint_data.size(); i += 3) {
    double lat = waypoint_data[i];
    double lon = waypoint_data[i + 1];
    double alt = waypoint_data[i + 2];

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

    // convert lat lon to local frame
    double dist_N, dist_E;
    latlon_diff(home_lat, home_lon, lat, lon, &dist_N, &dist_E);

    // add to waypoints
    Vec3 nwu{dist_N, -dist_E, alt};
    this->waypoints.push_back(nwu);
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
  Vec3 last_wp = this->waypoints.front();
  for (size_t i = 1; i < this->waypoints.size(); i++) {
    // calculate distance between current and last waypoint
    Vec3 wp = this->waypoints[i];

    // check distance
    if ((last_wp - wp).norm() > this->threshold_waypoint_gap) {
      LOG_ERROR(EDISTLATLON, (int) i + 1, this->threshold_waypoint_gap);
      return -2;
    }

    // update last waypoint
    last_wp = wp;
  }

  return 0;
}

Vec3 Mission::closestPoint(const Vec3 &position) {
  // calculate closest point
  Vec3 v1 = position - this->wp_start;
  Vec3 v2 = this->wp_end - this->wp_start;
  double t = v1.dot(v2) / v2.squaredNorm();

  // make sure the point is between wp_start and wp_end
  if (t < 0) {
    return this->wp_start;
  } else if (t > 1) {
    return this->wp_end;
  }

  return this->wp_start + t * v2;
}

int Mission::pointLineSide(const Vec3 &position) {
  Vec3 a = this->wp_start;
  Vec3 b = this->wp_end;
  Vec3 c = position;
  double s = ((b(0) - a(0)) * (c(1) - a(1)) - (b(1) - a(1)) * (c(0) - a(0)));

  // position is colinear with waypoint track
  if (fltcmp(s, 0.0) == 0) {
    return 0;
  }

  // position is left of waypoint track
  if (s > 0.0) {
    return 1;
  }

  // position is right of waypoint track
  return -1;
}

double Mission::crossTrackError(const Vec3 &position, int mode) {
  Vec3 BA = this->wp_start - position;
  Vec3 BC = this->wp_start - this->wp_end;

  // only calculate horizontal crosstrack error by setting z to 0
  if (mode == CTRACK_HORIZ) {
    BA(2) = 0.0;
    BC(2) = 0.0;
  }

  // crosstrack error
  double error = (BA.cross(BC)).norm() / BC.norm();

  // check which side the point is on
  int side = this->pointLineSide(position);

  return error * side;
}

Vec3 Mission::waypointTangentUnitVector() {
  Vec3 u = this->wp_start;
  Vec3 v = this->wp_end;
  return (v - u) / (v - u).norm();
}

double Mission::waypointHeading() {
  double dx = this->wp_end(0) - this->wp_start(0);
  double dy = this->wp_end(1) - this->wp_start(1);
  return atan2(dy, dx);
}

Vec3 Mission::waypointInterpolate(const Vec3 &position, const double r) {
  // get closest point
  Vec3 pt_on_line = this->closestPoint(position);

  // calculate waypoint between wp_start and wp_end
  Vec3 v = this->wp_end - this->wp_start;
  Vec3 u = v / v.norm();
  return pt_on_line + r * u;
}

int Mission::waypointReached(const Vec3 &position) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // calculate distance to waypoint
  Vec3 x = this->wp_end - position;
  double dist = x.norm();

  // waypoint reached?
  if (dist > this->threshold_waypoint_reached) {
    return 0;
  } else {
    return 1;
  }
}

int Mission::update(const Vec3 &position, Vec3 &waypoint) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // waypoint reached? get new wp_start and wp_end
  if (this->waypointReached(position)) {
    if (this->waypoints.size() > 2) {
      this->waypoints.pop_front();
      this->wp_start = this->waypoints.at(0);
      this->wp_end = this->waypoints.at(1);
    } else {
      return -2;
    }
  }

  // interpolate new waypoint
  waypoint = this->waypointInterpolate(position, this->look_ahead_dist);

  return 0;
}

}  // namespace atl
