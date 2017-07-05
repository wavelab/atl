#include "atl/control/velocity_controller.hpp"


namespace atl {

VelocityController::VelocityController(void) {
  this->configured = false;

  this->dt = 0.0;
  this->vx_controller = PID(0.0, 0.0, 0.0);
  this->vy_controller = PID(0.0, 0.0, 0.0);
  this->vz_controller = PID(0.0, 0.0, 0.0);

  this->roll_limit[0] = 0.0;
  this->roll_limit[1] = 0.0;
  this->pitch_limit[0] = 0.0;
  this->pitch_limit[1] = 0.0;
  this->throttle_limit[0] = 0.0;
  this->throttle_limit[1] = 0.0;

  this->setpoints << 0.0, 0.0, 0.0;
  this->outputs << 0.0, 0.0, 0.0, 0.0;
  this->att_cmd = AttitudeCommand();
}

int VelocityController::configure(std::string config_file) {
  ConfigParser parser;

  // load config
  // clang-format off
  parser.addParam<double>("vx_controller.pitch_min", &this->pitch_limit[0]);
  parser.addParam<double>("vx_controller.pitch_max", &this->pitch_limit[1]);
  parser.addParam<double>("vx_controller.k_p", &this->vx_controller.k_p);
  parser.addParam<double>("vx_controller.k_i", &this->vx_controller.k_i);
  parser.addParam<double>("vx_controller.k_d", &this->vx_controller.k_d);

  parser.addParam<double>("vy_controller.roll_min", &this->roll_limit[0]);
  parser.addParam<double>("vy_controller.roll_max", &this->roll_limit[1]);
  parser.addParam<double>("vy_controller.k_p", &this->vy_controller.k_p);
  parser.addParam<double>("vy_controller.k_i", &this->vy_controller.k_i);
  parser.addParam<double>("vy_controller.k_d", &this->vy_controller.k_d);

  parser.addParam<double>("vz_controller.throttle_min", &this->throttle_limit[0]);
  parser.addParam<double>("vz_controller.throttle_max", &this->throttle_limit[1]);
  parser.addParam<double>("vz_controller.k_p", &this->vz_controller.k_p);
  parser.addParam<double>("vz_controller.k_i", &this->vz_controller.k_i);
  parser.addParam<double>("vz_controller.k_d", &this->vz_controller.k_d);
  // clang-format on

  // convert roll and pitch limits from degrees to radians
  this->roll_limit[0] = deg2rad(this->roll_limit[0]);
  this->roll_limit[1] = deg2rad(this->roll_limit[1]);
  this->pitch_limit[0] = deg2rad(this->pitch_limit[0]);
  this->pitch_limit[1] = deg2rad(this->pitch_limit[1]);

  if (parser.load(config_file) != 0) {
    return -1;
  }

  this->configured = true;
  return 0;
}

Vec4 VelocityController::calculate(Vec3 setpoints, Vec3 actual, double dt) {
  // check rate
  this->dt += dt;
  if (this->dt < 0.01) {
    return this->outputs;
  }

  // calculate errors
  Vec3 errors;
  errors(0) = setpoints(0) - actual(0);
  errors(1) = setpoints(1) - actual(1);
  errors(2) = setpoints(2) - actual(2);

  // roll, pitch, yaw and throttle (assuming NWU frame)
  // clang-format off
  double r = -this->vy_controller.calculate(errors(1), 0.0, this->dt);
  double p = this->vx_controller.calculate(errors(0), 0.0, this->dt);
  double y = 0.0;
  double t = this->vz_controller.calculate(errors(2), 0.0, this->dt);
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
  Vec4 outputs{r, p, y, t};

  // keep track of setpoints and outputs
  this->setpoints = setpoints;
  this->outputs = outputs;
  this->dt = 0.0;

  return outputs;
}

void VelocityController::reset(void) {
  this->vx_controller.reset();
  this->vy_controller.reset();
  this->vz_controller.reset();
}

void VelocityController::printOutputs(void) {
  double r = rad2deg(this->outputs(0));
  double p = rad2deg(this->outputs(1));
  double t = this->outputs(3);

  std::cout << "roll: " << std::setprecision(2) << r << "\t";
  std::cout << "pitch: " << std::setprecision(2) << p << "\t";
  std::cout << "throttle: " << std::setprecision(2) << t << std::endl;
}

void VelocityController::printErrors(void) {
  double p = this->vx_controller.error_p;
  double i = this->vx_controller.error_i;
  double d = this->vx_controller.error_d;

  std::cout << "x_controller: " << std::endl;
  std::cout << "\terror_p: " << std::setprecision(2) << p << "\t";
  std::cout << "\terror_i: " << std::setprecision(2) << i << "\t";
  std::cout << "\terror_d: " << std::setprecision(2) << d << std::endl;

  p = this->vy_controller.error_p;
  i = this->vy_controller.error_i;
  d = this->vy_controller.error_d;

  std::cout << "y_controller: " << std::endl;
  std::cout << "\terror_p: " << std::setprecision(2) << p << "\t";
  std::cout << "\terror_i: " << std::setprecision(2) << i << "\t";
  std::cout << "\terror_d: " << std::setprecision(2) << d << std::endl;

  p = this->vz_controller.error_p;
  i = this->vz_controller.error_i;
  d = this->vz_controller.error_d;

  std::cout << "z_controller: " << std::endl;
  std::cout << "\terror_p: " << std::setprecision(2) << p << "\t";
  std::cout << "\terror_i: " << std::setprecision(2) << i << "\t";
  std::cout << "\terror_d: " << std::setprecision(2) << d << std::endl;
}

}  // namespace atl
