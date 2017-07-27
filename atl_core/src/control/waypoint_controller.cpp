#include "atl/control/waypoint_controller.hpp"

namespace atl {

int WaypointController::configure(const std::string &config_file) {
  ConfigParser parser;

  // load config
  parser.addParam("vx_controller.k_p", &this->vx_k_p);
  parser.addParam("vx_controller.k_i", &this->vx_k_i);
  parser.addParam("vx_controller.k_d", &this->vx_k_d);

  parser.addParam("vy_controller.k_p", &this->vy_k_p);
  parser.addParam("vy_controller.k_i", &this->vy_k_i);
  parser.addParam("vy_controller.k_d", &this->vy_k_d);

  parser.addParam("vz_controller.k_p", &this->vz_k_p);
  parser.addParam("vz_controller.k_i", &this->vz_k_i);
  parser.addParam("vz_controller.k_d", &this->vz_k_d);

  parser.addParam("roll_limit.min", &this->roll_limit[0]);
  parser.addParam("roll_limit.max", &this->roll_limit[1]);

  parser.addParam("pitch_limit.min", &this->pitch_limit[0]);
  parser.addParam("pitch_limit.max", &this->pitch_limit[1]);

  parser.addParam("throttle_limit.min", &this->throttle_limit[0]);
  parser.addParam("throttle_limit.max", &this->throttle_limit[1]);
  if (parser.load(config_file) != 0) {
    return -1;
  }

  // convert roll and pitch limits from degrees to radians
  this->roll_limit[0] = deg2rad(this->roll_limit[0]);
  this->roll_limit[1] = deg2rad(this->roll_limit[1]);
  this->pitch_limit[0] = deg2rad(this->pitch_limit[0]);
  this->pitch_limit[1] = deg2rad(this->pitch_limit[1]);

  this->configured = true;
  return 0;
}

Vec3 WaypointController::closestPoint(const Vec3 &position,
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

int WaypointController::pointLineSide(const Vec3 &wp_start,
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

double WaypointController::crossTrackError(const Vec3 &wp_start,
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

Vec3 WaypointController::waypointTangentUnitVector(const Vec3 &wp_start,
                                                   const Vec3 &wp_end) {
  return (wp_end - wp_start) / (wp_end - wp_start).norm();
}

double WaypointController::waypointHeading(const Vec3 &wp_start,
                                           const Vec3 &wp_end) {
  Vec3 u = wp_end - wp_start;
  Vec3 v = Vec3{1.0, 0.0, 0.0} - wp_start;

  // make sure we're only calculating horizontal angle between two vectors
  u(2) = 0.0;
  v(2) = 0.0;

  // double heading = acos(u.dot(v) / (u.norm().dot(v.norm())));
  double heading = 0.0;

  return heading;
}

Vec3 WaypointController::calculateWaypoint(const Vec3 &position,
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

int WaypointController::waypointReached(const Vec3 &position,
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

int WaypointController::waypointUpdate(const Vec3 &position, Vec3 &waypoint) {
  // pre-check
  if (this->configured == false) {
    return -1;
  }

  // waypoint reached? get new wp_start and wp_end
  if (this->waypointReached(position, this->wp_end, this->wp_threshold)) {
    if (this->waypoints.size() > 2) {
      this->waypoints.pop_front();
      this->wp_start = this->waypoints.at(0);
      this->wp_end = this->waypoints.at(1);
    } else {
      return -2;
    }
  }

  // calculate new waypoint
  waypoint = this->calculateWaypoint(
    position, this->look_ahead_dist, this->wp_start, this->wp_end);

  return 0;
}

int WaypointController::update(const Pose &pose,
                               const Vec3 &vel,
                               const double dt,
                               Vec4 &u) {
  // check rate
  this->dt += dt;
  if (this->dt < 0.01) {
    u = this->outputs;
    return 0;
  }

  // current waypoint
  Vec3 waypoint;
  int retval = this->waypointUpdate(pose.position, waypoint);
  if (retval != 0) {
    u = this->outputs;
    return retval;
  }

  // roll
  double error_ct =
    this->crossTrackError(this->wp_start, this->wp_end, pose.position);
  double r = this->vy_k_p * error_ct;
  r += this->vy_k_i * this->error_ct_sum;
  r += this->vy_k_d * (error_ct - this->error_ct_prev) / this->dt;
  this->error_ct_sum += error_ct;
  this->error_ct_prev = error_ct;

  // pitch
  Vec3 tu = this->waypointTangentUnitVector(this->wp_end, this->wp_start);
  double error_at = vel_desired - vel.dot(tu);
  double p = this->vx_k_p * error_at;
  p += this->vx_k_i * this->error_at_sum;
  p += this->vx_k_d * (error_at - this->error_at_prev) / this->dt;
  this->error_at_sum += error_at;
  this->error_at_prev = error_at;

  // yaw
  Vec3 euler;
  quat2euler(pose.orientation, 321, euler);
  double yaw_setpoint = this->waypointHeading(this->wp_start, this->wp_end);
  double y = this->yaw_k_p * (yaw_setpoint - euler(2));

  // throttle
  double error_z = waypoint(2) - pose.position(2);
  double t = this->vz_k_p * error_z;
  t += this->vz_k_i * error_z_sum;
  t += this->vz_k_d * (error_z - this->vz_error_prev) / this->dt;
  t /= fabs(cos(r) * cos(p));  // adjust throttle for roll and pitch

  // limit roll, pitch, throttle
  r = (r < this->roll_limit[0]) ? this->roll_limit[0] : r;
  r = (r > this->roll_limit[1]) ? this->roll_limit[1] : r;
  p = (p < this->pitch_limit[0]) ? this->pitch_limit[0] : p;
  p = (p > this->pitch_limit[1]) ? this->pitch_limit[1] : p;
  t = (t < this->throttle_limit[0]) ? this->throttle_limit[0] : t;
  t = (t > this->throttle_limit[1]) ? this->throttle_limit[1] : t;

  // keep track of setpoints and outputs
  this->outputs << r, p, y, t;
  this->dt = 0.0;

  return 0;
}

}  // namespace atl
