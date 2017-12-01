#include "atl/control/tracking_controller.hpp"

namespace atl {

int TrackingController::configure(const std::string &config_file) {
  ConfigParser parser;

  // load config
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

  parser.addParam("throttle_controller.k_p", &this->z_controller.k_p);
  parser.addParam("throttle_controller.k_i", &this->z_controller.k_i);
  parser.addParam("throttle_controller.k_d", &this->z_controller.k_d);
  parser.addParam("throttle_controller.hover_throttle", &this->hover_throttle);

  parser.addParam("track_offset", &this->track_offset);
  if (parser.load(config_file) != 0) {
    return -1;
  }

  // convert roll and pitch limits from degrees to radians
  this->roll_limit[0] = deg2rad(this->roll_limit[0]);
  this->roll_limit[1] = deg2rad(this->roll_limit[1]);
  this->pitch_limit[0] = deg2rad(this->pitch_limit[0]);
  this->pitch_limit[1] = deg2rad(this->pitch_limit[1]);

  // initialize setpoints to zero
  this->setpoints << 0.0, 0.0, 0.0;
  this->outputs << 0.0, 0.0, 0.0, 0.0;

  this->configured = true;
  return 0;
}

AttitudeCommand TrackingController::update(const Vec3 &errors_B,
                                           const double yaw_W,
                                           const double dt) {
  // check rate
  this->dt += dt;
  if (this->dt < 0.01) {
    return this->outputs;
  }

  // add offsets
  Vec3 errors = errors_B + this->track_offset;

  // roll, pitch, yaw and throttle (assuming NWU frame)
  double r = -this->y_controller.update(errors(1), 0.0, this->dt);
  double p = this->x_controller.update(errors(0), 0.0, this->dt);
  double y = yaw_W;
  double t = this->hover_throttle;
  t += this->z_controller.update(errors(2), 0.0, this->dt);
  t /= fabs(cos(r) * cos(p)); // adjust throttle for roll and pitch

  // limit roll, pitch
  r = (r < this->roll_limit[0]) ? this->roll_limit[0] : r;
  r = (r > this->roll_limit[1]) ? this->roll_limit[1] : r;
  p = (p < this->pitch_limit[0]) ? this->pitch_limit[0] : p;
  p = (p > this->pitch_limit[1]) ? this->pitch_limit[1] : p;

  // limit throttle
  t = (t < 0) ? 0.0 : t;
  t = (t > 1.0) ? 1.0 : t;

  // keep track of setpoints and outputs
  this->setpoints = errors;
  this->outputs << r, p, y, t;
  this->dt = 0.0;

  return AttitudeCommand(this->outputs);
}

AttitudeCommand TrackingController::update(const Vec3 &target_pos_B,
                                           const Vec3 &pos_W,
                                           const Vec3 &pos_prev_W,
                                           const double yaw_W,
                                           const double dt) {
  Vec3 errors{target_pos_B(0), target_pos_B(1), pos_prev_W(2) - pos_W(2)};
  return this->update(errors, yaw_W, dt);
}

void TrackingController::reset() {
  this->x_controller.reset();
  this->y_controller.reset();
  this->z_controller.reset();
}

void TrackingController::printOutputs() {
  double r, p, t;

  r = rad2deg(this->outputs(0));
  p = rad2deg(this->outputs(1));
  t = this->outputs(3);

  std::cout << "roll: " << std::setprecision(2) << r << "\t";
  std::cout << "pitch: " << std::setprecision(2) << p << "\t";
  std::cout << "throttle: " << std::setprecision(2) << t << std::endl;
}

void TrackingController::printErrors() {
  double p = this->x_controller.error_p;
  double i = this->x_controller.error_i;
  double d = this->x_controller.error_d;

  std::cout << "x_controller: " << std::endl;
  std::cout << "\terror_p: " << std::setprecision(2) << p << "\t";
  std::cout << "\terror_i: " << std::setprecision(2) << i << "\t";
  std::cout << "\terror_d: " << std::setprecision(2) << d << std::endl;

  p = this->y_controller.error_p;
  i = this->y_controller.error_i;
  d = this->y_controller.error_d;

  std::cout << "y_controller: " << std::endl;
  std::cout << "\terror_p: " << std::setprecision(2) << p << "\t";
  std::cout << "\terror_i: " << std::setprecision(2) << i << "\t";
  std::cout << "\terror_d: " << std::setprecision(2) << d << std::endl;

  p = this->z_controller.error_p;
  i = this->z_controller.error_i;
  d = this->z_controller.error_d;

  std::cout << "z_controller: " << std::endl;
  std::cout << "\terror_p: " << std::setprecision(2) << p << "\t";
  std::cout << "\terror_i: " << std::setprecision(2) << i << "\t";
  std::cout << "\terror_d: " << std::setprecision(2) << d << std::endl;
}

} // namespace atl
