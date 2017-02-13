#include "awesomo_core/control/tracking_controller.hpp"


namespace awesomo {

TrackingController::TrackingController(void) {
  this->configured = false;

  this->pctrl_dt = 0.0;
  this->x_controller = PID(0.0, 0.0, 0.0);
  this->y_controller = PID(0.0, 0.0, 0.0);
  this->z_controller = PID(0.0, 0.0, 0.0);
  this->hover_throttle = 0.0;

  this->vctrl_dt = 0.0;
  this->vx_controller = PID(0.0, 0.0, 0.0);
  this->vy_controller = PID(0.0, 0.0, 0.0);
  this->vz_controller = PID(0.0, 0.0, 0.0);

  this->roll_limit[0] = 0.0;
  this->roll_limit[1] = 0.0;
  this->pitch_limit[0] = 0.0;
  this->pitch_limit[1] = 0.0;
  this->throttle_limit[0] = 0.0;
  this->throttle_limit[1] = 0.0;
  this->track_offset << 0.0, 0.0, 0.0;

  this->pctrl_setpoints << 0.0, 0.0, 0.0;
  this->pctrl_outputs << 0.0, 0.0, 0.0, 0.0;
  this->vctrl_setpoints << 0.0, 0.0, 0.0;
  this->vctrl_outputs << 0.0, 0.0, 0.0, 0.0;
  this->att_cmd = AttitudeCommand();
}

int TrackingController::configure(std::string config_file) {
  ConfigParser parser;

  // load config
  // clang-format off
  parser.addParam<double>("roll_controller.k_p", &this->y_controller.k_p);
  parser.addParam<double>("roll_controller.k_i", &this->y_controller.k_i);
  parser.addParam<double>("roll_controller.k_d", &this->y_controller.k_d);

  parser.addParam<double>("pitch_controller.k_p", &this->x_controller.k_p);
  parser.addParam<double>("pitch_controller.k_i", &this->x_controller.k_i);
  parser.addParam<double>("pitch_controller.k_d", &this->x_controller.k_d);

  parser.addParam<double>("throttle_controller.k_p", &this->z_controller.k_p);
  parser.addParam<double>("throttle_controller.k_i", &this->z_controller.k_i);
  parser.addParam<double>("throttle_controller.k_d", &this->z_controller.k_d);
  parser.addParam<double>("throttle_controller.hover_throttle", &this->hover_throttle);

  parser.addParam<double>("vx_controller.k_p", &this->vx_controller.k_p);
  parser.addParam<double>("vx_controller.k_i", &this->vx_controller.k_i);
  parser.addParam<double>("vx_controller.k_d", &this->vx_controller.k_d);

  parser.addParam<double>("vy_controller.k_p", &this->vy_controller.k_p);
  parser.addParam<double>("vy_controller.k_i", &this->vy_controller.k_i);
  parser.addParam<double>("vy_controller.k_d", &this->vy_controller.k_d);

  parser.addParam<double>("vz_controller.k_p", &this->vz_controller.k_p);
  parser.addParam<double>("vz_controller.k_i", &this->vz_controller.k_i);
  parser.addParam<double>("vz_controller.k_d", &this->vz_controller.k_d);

  parser.addParam<double>("roll_limit.min", &this->roll_limit[0]);
  parser.addParam<double>("roll_limit.max", &this->roll_limit[1]);

  parser.addParam<double>("pitch_limit.min", &this->pitch_limit[0]);
  parser.addParam<double>("pitch_limit.max", &this->pitch_limit[1]);

  parser.addParam<double>("throttle_limit.min", &this->throttle_limit[0]);
  parser.addParam<double>("throttle_limit.max", &this->throttle_limit[1]);

  parser.addParam<Vec3>("track_offset", &this->track_offset);
  // clang-format on

  if (parser.load(config_file) != 0) {
    return -1;
  }

  // convert roll and pitch limits from degrees to radians
  this->roll_limit[0] = deg2rad(this->roll_limit[0]);
  this->roll_limit[1] = deg2rad(this->roll_limit[1]);
  this->pitch_limit[0] = deg2rad(this->pitch_limit[0]);
  this->pitch_limit[1] = deg2rad(this->pitch_limit[1]);

  // initialize setpoints to zero
  this->pctrl_setpoints << 0.0, 0.0, 0.0;
  this->pctrl_outputs << 0.0, 0.0, 0.0, 0.0;
  this->vctrl_setpoints << 0.0, 0.0, 0.0;
  this->vctrl_outputs << 0.0, 0.0, 0.0, 0.0;

  this->configured = true;
  return 0;
}

Vec4 TrackingController::calculatePositionErrors(Vec3 errors,
                                                 double yaw,
                                                 double dt) {
  double r, p, y, t;
  Vec3 euler;
  Mat3 R;

  // check rate
  this->pctrl_dt += dt;
  if (this->pctrl_dt < 0.01) {
    return this->pctrl_outputs;
  }

  // add offsets
  errors = errors + this->track_offset;

  // roll, pitch, yaw and throttle (assuming NWU frame)
  // clang-format off
  r = -this->y_controller.calculate(errors(1), 0.0, this->pctrl_dt);
  p = this->x_controller.calculate(errors(0), 0.0, this->pctrl_dt);
  y = yaw;
  t = this->hover_throttle + this->z_controller.calculate(errors(2), 0.0, this->pctrl_dt);
  t /= fabs(cos(r) * cos(p));  // adjust throttle for roll and pitch
  // clang-format o

  // limit roll, pitch
  r = (r < this->roll_limit[0]) ? this->roll_limit[0] : r;
  r = (r > this->roll_limit[1]) ? this->roll_limit[1] : r;
  p = (p < this->pitch_limit[0]) ? this->pitch_limit[0] : p;
  p = (p > this->pitch_limit[1]) ? this->pitch_limit[1] : p;

  // limit throttle
  t = (t < 0) ? 0.0 : t;
  t = (t > 1.0) ? 1.0 : t;

  // keep track of setpoints and outputs
  this->pctrl_setpoints = errors;
  this->pctrl_outputs << r, p, y, t;
  this->pctrl_dt = 0.0;

  return pctrl_outputs;
}

Vec4 TrackingController::calculateVelocityErrors(Vec3 errors,
                                                 double yaw,
                                                 double dt) {
  double r, p, y, t;
  Vec3 euler;
  Mat3 R;

  // check rate
  this->vctrl_dt += dt;
  if (this->vctrl_dt < 0.01) {
    return this->vctrl_outputs;
  }

  // roll, pitch, yaw and throttle (assuming NWU frame)
  // clang-format off
  r = -this->vy_controller.calculate(errors(1), 0.0, this->vctrl_dt);
  p = this->vx_controller.calculate(errors(0), 0.0, this->vctrl_dt);
  y = 0.0;
  t = this->vz_controller.calculate(errors(2), 0.0, this->vctrl_dt);
  t /= fabs(cos(r) * cos(p));  // adjust throttle for roll and pitch
  // clang-format o

  // limit roll, pitch
  r = (r < this->roll_limit[0]) ? this->roll_limit[0] : r;
  r = (r > this->roll_limit[1]) ? this->roll_limit[1] : r;
  p = (p < this->pitch_limit[0]) ? this->pitch_limit[0] : p;
  p = (p > this->pitch_limit[1]) ? this->pitch_limit[1] : p;

  // // limit throttle
  // t = (t < 0) ? 0.0 : t;
  // t = (t > 1.0) ? 1.0 : t;

  // keep track of setpoints and outputs
  this->vctrl_setpoints = errors;
  this->vctrl_outputs << r, p, y, t;
  this->vctrl_dt = 0.0;

  return this->vctrl_outputs;
}

AttitudeCommand TrackingController::calculate(Vec3 pos_errors,
                                              Vec3 vel_errors,
                                              double yaw,
                                              double dt) {
  this->calculatePositionErrors(pos_errors, yaw, dt);
  this->calculateVelocityErrors(vel_errors, yaw, dt);
  this->att_cmd = AttitudeCommand(this->pctrl_outputs + this->vctrl_outputs);
  return this->att_cmd;
}

AttitudeCommand TrackingController::calculate(Vec3 target_pos_bf,
                                              Vec3 target_vel_bf,
                                              Vec3 pos,
                                              Vec3 pos_prev,
                                              double yaw,
                                              double dt) {
  Vec3 perrors, verrors;

  perrors(0) = target_pos_bf(0);
  perrors(1) = target_pos_bf(1);
  perrors(2) = pos_prev(2) - pos(2);

  verrors(0) = target_vel_bf(0);
  verrors(1) = target_vel_bf(1);
  verrors(2) = 0.0;

  return this->calculate(perrors, verrors, yaw, dt);
}

void TrackingController::reset(void) {
  this->x_controller.reset();
  this->y_controller.reset();
  this->z_controller.reset();

  this->vx_controller.reset();
  this->vy_controller.reset();
  this->vz_controller.reset();
}

void TrackingController::printOutputs(void) {
  double r, p, t;

  r = rad2deg(this->pctrl_outputs(0));
  p = rad2deg(this->pctrl_outputs(1));
  t = this->pctrl_outputs(3);

  std::cout << "roll: " << std::setprecision(2) << r << "\t";
  std::cout << "pitch: " << std::setprecision(2) << p << "\t";
  std::cout << "throttle: " << std::setprecision(2) << t << std::endl;
}

void TrackingController::printErrors(void) {
  double p, i, d;

  p = this->x_controller.error_p;
  i = this->x_controller.error_i;
  d = this->x_controller.error_d;

  std::cout << "x_controller: " << std::endl;
  std::cout << "\terror_p: " << std::setprecision(2) << p << "\t";
  std::cout << "\terror_i: " << std::setprecision(2) << i << "\t";
  std::cout << "\terror_d: " << std::setprecision(2) << i << std::endl;

  p = this->y_controller.error_p;
  i = this->y_controller.error_i;
  d = this->y_controller.error_d;

  std::cout << "y_controller: " << std::endl;
  std::cout << "\terror_p: " << std::setprecision(2) << p << "\t";
  std::cout << "\terror_i: " << std::setprecision(2) << i << "\t";
  std::cout << "\terror_d: " << std::setprecision(2) << i << std::endl;

  p = this->z_controller.error_p;
  i = this->z_controller.error_i;
  d = this->z_controller.error_d;

  std::cout << "z_controller: " << std::endl;
  std::cout << "\terror_p: " << std::setprecision(2) << p << "\t";
  std::cout << "\terror_i: " << std::setprecision(2) << i << "\t";
  std::cout << "\terror_d: " << std::setprecision(2) << i << std::endl;
}

}  // end of awesomo namespace
