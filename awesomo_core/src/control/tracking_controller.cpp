#include "awesomo_core/control/tracking_controller.hpp"


namespace awesomo {

TrackingController::TrackingController(void) {
  this->configured = false;

  this->dt = 0.0;
  this->x_controller = PID(0.0, 0.0, 0.0);
  this->y_controller = PID(0.0, 0.0, 0.0);
  this->z_controller = PID(0.0, 0.0, 0.0);

  this->roll_limit[0] = 0.0;
  this->roll_limit[1] = 0.0;
  this->pitch_limit[0] = 0.0;
  this->pitch_limit[1] = 0.0;
  this->hover_throttle = 0.0;

  this->setpoints << 0.0, 0.0, 0.0;
  this->outputs << 0.0, 0.0, 0.0, 0.0;
  this->att_cmd = AttitudeCommand();
}

int TrackingController::configure(std::string config_file) {
  ConfigParser parser;

  // load config
  parser.addParam<double>("roll_controller.k_p", &this->y_controller.k_p);
  parser.addParam<double>("roll_controller.k_i", &this->y_controller.k_i);
  parser.addParam<double>("roll_controller.k_d", &this->y_controller.k_d);
  parser.addParam<double>("roll_controller.min", &this->roll_limit[0]);
  parser.addParam<double>("roll_controller.max", &this->roll_limit[1]);

  parser.addParam<double>("pitch_controller.k_p", &this->x_controller.k_p);
  parser.addParam<double>("pitch_controller.k_i", &this->x_controller.k_i);
  parser.addParam<double>("pitch_controller.k_d", &this->x_controller.k_d);
  parser.addParam<double>("pitch_controller.min", &this->pitch_limit[0]);
  parser.addParam<double>("pitch_controller.max", &this->pitch_limit[1]);

  parser.addParam<double>("throttle_controller.k_p", &this->z_controller.k_p);
  parser.addParam<double>("throttle_controller.k_i", &this->z_controller.k_i);
  parser.addParam<double>("throttle_controller.k_d", &this->z_controller.k_d);
  parser.addParam<double>("throttle_controller.hover_throttle",
                          &this->hover_throttle);

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

Vec4 TrackingController::calculate(Vec3 errors, double yaw, double dt) {
  double r, p, y, t;
  Vec3 euler;
  Vec4 outputs;
  Mat3 R;

  // check rate
  this->dt += dt;
  if (this->dt < 0.01) {
    return this->outputs;
  }

  // roll, pitch, yaw and throttle (assuming NWU frame)
  // clang-format off
  r = -this->y_controller.calculate(errors(1), 0.0, this->dt);
  p = this->x_controller.calculate(errors(0), 0.0, this->dt);
  y = yaw;
  t = this->hover_throttle + this->z_controller.calculate(errors(2), 0.0, this->dt);
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

  // set outputs
  outputs << r, p, y, t;

  // keep track of setpoints and outputs
  this->setpoints = setpoints;
  this->outputs = outputs;
  this->dt = 0.0;

  return outputs;
}

void TrackingController::reset(void) {
  this->x_controller.reset();
  this->y_controller.reset();
  this->z_controller.reset();
}

void TrackingController::printOutputs(void) {
  double r, p, t;

  r = rad2deg(this->outputs(0));
  p = rad2deg(this->outputs(1));
  t = this->outputs(3);

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
