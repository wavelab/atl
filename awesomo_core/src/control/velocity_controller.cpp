#include "awesomo_core/control/velocity_controller.hpp"


namespace awesomo {

VelocityController::VelocityController(void) {
  this->configured = false;

  this->vx_controller = PID(0.0, 0.0, 0.0);
  this->vy_controller = PID(0.0, 0.0, 0.0);
  this->vz_controller = PID(0.0, 0.0, 0.0);

  this->hover_throttle = 0.0;

  this->roll_limit[0] = 0.0;
  this->roll_limit[1] = 0.0;

  this->pitch_limit[0] = 0.0;
  this->pitch_limit[1] = 0.0;

  this->setpoints[0] = 0;
  this->setpoints[1] = 0;
  this->setpoints[2] = 0;

  this->outputs[0] = 0;
  this->outputs[1] = 0;
  this->outputs[2] = 0;
  this->outputs[3] = 0;
}

int VelocityController::configure(std::string config_file) {
  try {
    YAML::Node config;
    YAML::Node roll_controller;
    YAML::Node pitch_controller;
    YAML::Node throttle_controller;

    // pre-check
    if (file_exists(config_file) == false) {
      return -1;
    }

    // setup
    config = YAML::LoadFile(config_file);
    roll_controller = config["roll_controller"];
    pitch_controller = config["pitch_controller"];
    throttle_controller = config["throttle_controller"];

    // clang-format off
    // vx controller
    this->vx_controller = PID(
      roll_controller["k_p"].as<float>(),
      roll_controller["k_i"].as<float>(),
      roll_controller["k_d"].as<float>()
    );
    this->roll_limit[0] = deg2rad(roll_controller["min"].as<float>());
    this->roll_limit[1] = deg2rad(roll_controller["max"].as<float>());

    // vy controller
    this->vy_controller = PID(
      pitch_controller["k_p"].as<float>(),
      pitch_controller["k_i"].as<float>(),
      pitch_controller["k_d"].as<float>()
    );
    this->pitch_limit[0] = deg2rad(pitch_controller["min"].as<float>());
    this->pitch_limit[1] = deg2rad(pitch_controller["max"].as<float>());

    // vz controller
    this->vz_controller = PID(
      throttle_controller["k_p"].as<float>(),
      throttle_controller["k_i"].as<float>(),
      throttle_controller["k_d"].as<float>()
    );
    this->hover_throttle = throttle_controller["hover_throttle"].as<float>();
    // clang-format on

  } catch (YAML::BadFile &ex) {
    throw;
    return -1;
  }

  this->configured = true;
  return 0;
}

VecX VelocityController::calculate(Vec3 setpoints,
                                   Vec3 actual,
                                   double yaw,
                                   double dt) {
  VecX outputs(4);
  double r, p, y, t;

  // roll, pitch, yaw and throttle (assuming ENU frame)
  r = this->vy_controller.calculate(setpoints(1), actual(1), dt);
  p = -this->vx_controller.calculate(setpoints(0), actual(0), dt);
  y = yaw;
  t = this->hover_throttle +
      this->vz_controller.calculate(setpoints(2), actual(2), dt);
  t /= fabs(cos(actual(1)) *
            cos(actual(0)));  // adjust throttle for roll and pitch

  // limit roll, pitch
  r = (r < this->roll_limit[0]) ? this->roll_limit[0] : r;
  r = (r > this->roll_limit[1]) ? this->roll_limit[1] : r;
  p = (p < this->pitch_limit[0]) ? this->pitch_limit[0] : p;
  p = (p > this->pitch_limit[1]) ? this->pitch_limit[1] : p;

  // limit yaw
  while (outputs(2) > deg2rad(360)) {
    y -= deg2rad(360);
  }
  while (y < 0) {
    y += deg2rad(360);
  }

  // limit throttle
  t = (t < 0) ? 0.0 : t;
  t = (t > 1.0) ? 1.0 : t;

  // set outputs
  outputs << r, p, y, t;

  // keep track of setpoints and outputs
  this->setpoints[0] = setpoints(0);
  this->setpoints[1] = setpoints(1);
  this->setpoints[2] = setpoints(2);

  this->outputs[0] = outputs(0);
  this->outputs[1] = outputs(1);
  this->outputs[2] = outputs(2);
  this->outputs[3] = outputs(3);

  return outputs;
}

void VelocityController::reset(void) {
  this->vx_controller.reset();
  this->vy_controller.reset();
  this->vz_controller.reset();
}

}  // end of awesomo namespace
