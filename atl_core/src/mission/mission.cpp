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

Vec3 Mission::closestPoint(const Vec3 &position,
                           const Vec3 &wp_start,
                           const Vec3 &wp_end) {
  // calculate closest point
  Vec3 v1 = position - wp_start;
  Vec3 v2 = wp_end - wp_start;
  double t = v1.dot(v2) / v2.squaredNorm();

  // make sure the point is between wp_start and wp_end
  if (t < 0) {
    return wp_start;
  } else if (t > 1) {
    return wp_end;
  }

  return wp_start + t * v2;
}

int Mission::pointLineSide(const Vec3 &wp_start,
                           const Vec3 &wp_end,
                           const Vec3 &position) {
  Vec3 a = wp_start;
  Vec3 b = wp_end;
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

double Mission::crossTrackError(const Vec3 &wp_start,
                                const Vec3 &wp_end,
                                const Vec3 &position,
                                int mode) {
  Vec3 BA = wp_start - position;
  Vec3 BC = wp_start - wp_end;

  // only calculate horizontal crosstrack error by setting z to 0
  if (mode == CTRACK_HORIZ) {
    BA(2) = 0.0;
    BC(2) = 0.0;
  }

  // crosstrack error
  double error = (BA.cross(BC)).norm() / BC.norm();

  // check which side the point is on
  int side = this->pointLineSide(wp_start, wp_end, position);

  return error * side;
}

Vec3 Mission::waypointTangentUnitVector(const Vec3 &wp_start,
                                        const Vec3 &wp_end) {
  return (wp_end - wp_start) / (wp_end - wp_start).norm();
}

double Mission::waypointHeading(const Vec3 &wp_start, const Vec3 &wp_end) {
  Vec3 u = wp_end - wp_start;
  Vec3 v = Vec3{1.0, 0.0, 0.0} - wp_start;

  // make sure we're only calculating horizontal angle between two vectors
  u(2) = 0.0;
  v(2) = 0.0;

  // double heading = acos(u.dot(v) / (u.norm().dot(v.norm())));
  double heading = 0.0;

  return heading;
}

Vec3 Mission::calculateWaypoint(const Vec3 &position,
                                const double r,
                                const Vec3 &wp_start,
                                const Vec3 &wp_end) {
  // get closest point
  Vec3 pt_on_line = this->closestPoint(position, wp_start, wp_end);

  // calculate waypoint between wp_start and wp_end
  Vec3 v = wp_end - wp_start;
  Vec3 u = v / v.norm();
  return pt_on_line + r * u;
}

int Mission::waypointReached(const Vec3 &position,
                             const Vec3 &waypoint,
                             const double threshold) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // calculate distance to waypoint
  Vec3 x = waypoint - position;
  double dist = x.norm();

  // waypoint reached?
  if (dist > threshold) {
    return 0;
  } else {
    return 1;
  }
}

}  // namespace atl
