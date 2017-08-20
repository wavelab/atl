#include "atl/control/landing_controller.hpp"

namespace atl {

int LandingController::configure(const std::string &config_file) {
  std::string traj_index_file;

  // load config
  ConfigParser parser;
  parser.addParam("roll_controller.k_p", &this->y_controller.k_p);
  parser.addParam("roll_controller.k_i", &this->y_controller.k_i);
  parser.addParam("roll_controller.k_d", &this->y_controller.k_d);
  parser.addParam("roll_controller.min", &this->roll_limit[0]);
  parser.addParam("roll_controller.max", &this->roll_limit[1]);

  parser.addParam("pitch_controller.k_p", &this->x_controller.k_p);
  parser.addParam("pitch_controller.k_i", &this->x_controller.k_i);
  parser.addParam("pitch_controller.k_d", &this->x_controller.k_d);
  parser.addParam("pitch_controller.min", &this->pitch_limit[0]);
  parser.addParam("pitch_controller.max", &this->pitch_limit[1]);

  parser.addParam("vz_controller.k_p", &this->vz_controller.k_p);
  parser.addParam("vz_controller.k_i", &this->vz_controller.k_i);
  parser.addParam("vz_controller.k_d", &this->vz_controller.k_d);
  parser.addParam("vz_controller.hover_throttle", &this->hover_throttle);
  parser.addParam("vz_controller.descent_velocity", &this->descent_velocity);
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

Vec4 LandingController::update(const Vec3 &pos_errors_bf,
                               const Vec3 &velocity_if,
                               const double yaw_setpoint,
                               const double dt) {
  // check rate
  this->dt += dt;
  if (this->dt < 0.01) {
    return this->outputs;
  }

  // roll, pitch, yaw and throttle (assuming NWU frame)
  double r = -this->y_controller.update(pos_errors_bf(1), this->dt);
  double p = this->x_controller.update(pos_errors_bf(0), this->dt);
  double y = yaw_setpoint;
  const double vz_error = this->descent_velocity - velocity_if(2);
  double t = this->hover_throttle;
  t += this->vz_controller.update(vz_error, this->dt);
  t /= fabs(cos(r) * cos(p)); // adjust throttle for roll and pitch

  // limit roll, pitch and throttle
  r = (r < this->roll_limit[0]) ? this->roll_limit[0] : r;
  r = (r > this->roll_limit[1]) ? this->roll_limit[1] : r;
  p = (p < this->pitch_limit[0]) ? this->pitch_limit[0] : p;
  p = (p > this->pitch_limit[1]) ? this->pitch_limit[1] : p;
  t = (t < 0) ? 0.0 : t;
  t = (t > 1.0) ? 1.0 : t;

  // keep track of setpoints and outputs
  this->setpoints = setpoints;
  this->outputs << r, p, y, t;
  this->dt = 0.0;

  return outputs;
}

void LandingController::reset() {
  this->x_controller.reset();
  this->y_controller.reset();
  this->vz_controller.reset();
}

void LandingController::printOutputs() {
  double r, p, t;

  r = rad2deg(this->outputs(0));
  p = rad2deg(this->outputs(1));
  t = this->outputs(3);

  std::cout << "roll: " << std::setprecision(2) << r << "\t";
  std::cout << "pitch: " << std::setprecision(2) << p << "\t";
  std::cout << "throttle: " << std::setprecision(2) << t << std::endl;
}

} // namespace atl
