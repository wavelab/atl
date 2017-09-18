#include "atl/control/waypoint_controller.hpp"

namespace atl {

int WaypointController::configure(const std::string &config_file) {
  ConfigParser parser;

  // load config
  parser.addParam("at_controller.k_p", &this->at_controller.k_p);
  parser.addParam("at_controller.k_i", &this->at_controller.k_i);
  parser.addParam("at_controller.k_d", &this->at_controller.k_d);
  parser.addParam("at_controller.min", &this->pitch_limit[0]);
  parser.addParam("at_controller.max", &this->pitch_limit[1]);

  parser.addParam("ct_controller.k_p", &this->ct_controller.k_p);
  parser.addParam("ct_controller.k_i", &this->ct_controller.k_i);
  parser.addParam("ct_controller.k_d", &this->ct_controller.k_d);
  parser.addParam("ct_controller.min", &this->roll_limit[0]);
  parser.addParam("ct_controller.max", &this->roll_limit[1]);

  parser.addParam("z_controller.k_p", &this->z_controller.k_p);
  parser.addParam("z_controller.k_i", &this->z_controller.k_i);
  parser.addParam("z_controller.k_d", &this->z_controller.k_d);
  parser.addParam("z_controller.hover_throttle", &this->hover_throttle);
  if (parser.load(config_file) != 0) {
    return -1;
  }

  // convert roll and pitch limits from degrees to radians
  this->roll_limit[0] = deg2rad(this->roll_limit[0]);
  this->roll_limit[1] = deg2rad(this->roll_limit[1]);
  this->pitch_limit[0] = deg2rad(this->pitch_limit[0]);
  this->pitch_limit[1] = deg2rad(this->pitch_limit[1]);

  // prepare blackbox file
  std::string blackbox_file = "/tmp/blackbox.dat";
  if (this->blackbox_enable) {
    if (blackbox_file == "") {
      LOG_ERROR("blackbox file is not set!");
      return -3;
    } else if (this->prepBlackbox(blackbox_file) != 0) {
      LOG_ERROR("Failed to open blackbox file at [%s]", blackbox_file.c_str());
      return -3;
    }

    if (this->blackbox_rate == FLT_MAX) {
      LOG_ERROR("blackbox rate is not set!");
      return -3;
    }
  }

  this->configured = true;
  return 0;
}

int WaypointController::prepBlackbox(const std::string &blackbox_file) {
  // setup
  this->blackbox.open(blackbox_file);
  if (!this->blackbox) {
    return -1;
  }

  // write header
  // clang-format off
  this->blackbox << "dt" << ",";
  this->blackbox << "x" << ",";
  this->blackbox << "y" << ",";
  this->blackbox << "z" << ",";
  this->blackbox << "wp_x" << ",";
  this->blackbox << "wp_y" << ",";
  this->blackbox << "wp_z";
  this->blackbox << std::endl;
  // clang-format on

  return 0;
}

int WaypointController::record(const Vec3 &pos, const Vec3 &waypoint) {
  // pre-check
  this->blackbox_dt += dt;
  if (this->blackbox_enable && this->blackbox_dt > this->blackbox_rate) {
    return 0;
  }

  // record
  this->blackbox << pos(0) << ",";
  this->blackbox << pos(1) << ",";
  this->blackbox << pos(2) << ",";
  this->blackbox << waypoint(0) << ",";
  this->blackbox << waypoint(1) << ",";
  this->blackbox << waypoint(2);
  this->blackbox << std::endl;
  this->blackbox_dt = 0.0;

  return 0;
}

double WaypointController::calcYawToWaypoint(const Vec3 &waypoint,
                                             const Vec3 &position) {
  // assume waypoints are in NWU inertial frame
  const double dx = waypoint(0) - position(0);
  const double dy = waypoint(1) - position(1);

  // calculate heading
  double heading = atan2(dy, dx);
  if (heading > M_PI) {
    heading -= 2 * M_PI;
  } else if (heading < -M_PI) {
    heading += 2 * M_PI;
  }

  return heading;
}

int WaypointController::update(Mission &mission,
                               const Pose &pose,
                               const Vec3 &vel,
                               const double dt) {
  // check rate
  this->dt += dt;
  if (this->dt < 0.01) {
    return 0;
  }

  // current waypoint
  Vec3 waypoint = Vec3::Zero();
  int retval = mission.update(pose.position, waypoint);
  if (retval != 0) {
    return retval;
  }

  // calculate waypoint relative to quadrotor
  Vec3 errors = T_P_W{pose.orientation} * Vec3{waypoint - pose.position};

  // calculate velocity relative to quadrotor
  const Vec3 vel_B = T_P_W{pose.orientation} * vel;

  // roll
  double r = -this->ct_controller.update(errors(1), this->dt);

  // pitch
  double error_forward = mission.desired_velocity - vel_B(0);
  double p = this->at_controller.update(error_forward, this->dt);

  // yaw
  double y = mission.waypointHeading();

  // throttle
  const double error_z = waypoint(2) - pose.position(2);
  double t = this->hover_throttle;
  t += this->z_controller.update(error_z, this->dt);
  t /= fabs(cos(r) * cos(p)); // adjust throttle for roll and pitch

  // limit roll, pitch and throttle
  r = (r < this->roll_limit[0]) ? this->roll_limit[0] : r;
  r = (r > this->roll_limit[1]) ? this->roll_limit[1] : r;
  p = (p < this->pitch_limit[0]) ? this->pitch_limit[0] : p;
  p = (p > this->pitch_limit[1]) ? this->pitch_limit[1] : p;
  t = (t < 0.0) ? 0.0 : t;
  t = (t > 1.0) ? 1.0 : t;

  // keep track of setpoints and outputs
  this->setpoints = waypoint;
  this->outputs << r, p, y, t;
  this->record(pose.position, waypoint);
  this->dt = 0.0;

  return 0;
}

void WaypointController::reset() {
  this->at_controller.reset();
  this->ct_controller.reset();
  this->z_controller.reset();
  this->yaw_controller.reset();
}

} // namespace atl
