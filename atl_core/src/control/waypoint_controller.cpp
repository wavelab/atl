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

Vec4 WaypointController::calculate(const Vec3 &pos_setpoints,
                                   const Vec3 &vel_setpoints,
                                   const double yaw_setpoint,
                                   const Pose &pose,
                                   const Vec3 &vel,
                                   const double dt) {
  // check rate
  this->dt += dt;
  if (this->dt < 0.01) {
    return this->outputs;
  }

  // calculate position errors
  Vec3 p_errors;
  Vec3 v_errors;
  target2bodyplanar(pos_setpoints, pose.position, pose.orientation, p_errors);
  target2bodyplanar(vel_setpoints, pose.position, pose.orientation, v_errors);

  // roll
  double r = this->vy_k_p * v_errors(1);
  r += this->vy_k_i * p_errors(1);
  r += this->vy_k_d * (v_errors(1) - this->vy_error_prev) / this->dt;
  r = -1 * r;

  // pitch
  double p = this->vx_k_p * v_errors(0);
  p += this->vx_k_i * p_errors(0);
  p += this->vx_k_d * (v_errors(0) - this->vx_error_prev) / this->dt;

  // yaw
  Vec3 euler;
  quat2euler(pose.orientation, 321, euler);
  double y = this->yaw_k_p * (yaw_setpoint - euler(2));

  // throttle
  double t = this->vz_k_p * v_errors(2);
  t += this->vz_k_i * p_errors(2);
  t += this->vz_k_d * (v_errors(2) - this->vz_error_prev) / this->dt;
  t /= fabs(cos(r) * cos(p));  // adjust throttle for roll and pitch

  // keep track of previous errors
  this->vx_error_prev = v_errors(0);
  this->vy_error_prev = v_errors(1);
  this->vz_error_prev = v_errors(2);

  // limit roll, pitch, throttle
  r = (r < this->roll_limit[0]) ? this->roll_limit[0] : r;
  r = (r > this->roll_limit[1]) ? this->roll_limit[1] : r;
  p = (p < this->pitch_limit[0]) ? this->pitch_limit[0] : p;
  p = (p > this->pitch_limit[1]) ? this->pitch_limit[1] : p;
  t = (t < this->throttle_limit[0]) ? this->throttle_limit[0] : t;
  t = (t > this->throttle_limit[1]) ? this->throttle_limit[1] : t;

  // keep track of setpoints and outputs
  this->pos_setpoints = pos_setpoints;
  this->vel_setpoints = vel_setpoints;
  this->outputs << r, p, y, t;
  this->dt = 0.0;

  return this->outputs;
}

}  // namespace atl
