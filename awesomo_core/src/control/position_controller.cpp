#include "awesomo_core/control/position_controller.hpp"


namespace awesomo {

PositionController::PositionController(void) {
  this->configured = false;

  this->x_controller = PID(0.0, 0.0, 0.0);
  this->y_controller = PID(0.0, 0.0, 0.0);
  this->z_controller = PID(0.0, 0.0, 0.0);

  this->hover_throttle = 0.0;

  this->roll_limit[0] = 0.0;
  this->roll_limit[1] = 0.0;

  this->pitch_limit[0] = 0.0;
  this->pitch_limit[1] = 0.0;

  this->setpoint_x = 0.0;
  this->setpoint_y = 0.0;
  this->setpoint_z = 0.0;

  this->output_roll = 0.0;
  this->output_pitch = 0.0;
  this->output_throttle = 0.0;
}

int PositionController::configure(std::string config_file) {
  try {
    YAML::Node config;
    YAML::Node roll_controller;
    YAML::Node pitch_controller;
    YAML::Node throttle_controller;

    std::cout << config_file << std::endl;

    config = YAML::LoadFile(config_file);
    roll_controller = config["roll_controller"];
    pitch_controller = config["pitch_controller"];
    throttle_controller = config["throttle_controller"];

    // clang-format off
    // roll controller
    this->x_controller = PID(
      roll_controller["k_p"].as<float>(),
      roll_controller["k_i"].as<float>(),
      roll_controller["k_d"].as<float>()
    );
    this->roll_limit[0] = deg2rad(roll_controller["min"].as<float>());
    this->roll_limit[1] = deg2rad(roll_controller["max"].as<float>());

    // pitch controller
    this->y_controller = PID(
      pitch_controller["k_p"].as<float>(),
      pitch_controller["k_i"].as<float>(),
      pitch_controller["k_d"].as<float>()
    );
    this->pitch_limit[0] = deg2rad(pitch_controller["min"].as<float>());
    this->pitch_limit[1] = deg2rad(pitch_controller["max"].as<float>());

    // throttle_controller
    this->z_controller = PID(
      throttle_controller["k_p"].as<float>(),
      throttle_controller["k_i"].as<float>(),
      throttle_controller["k_d"].as<float>()
    );
    this->hover_throttle = throttle_controller["hover_throttle"].as<float>();
    // clang-format on

  } catch (std::exception &ex) {
    std::cout << ex.what();
    return -1;
  }

  this->configured = true;
  return 0;
}

VecX PositionController::calculate(VecX setpoints,
                                   VecX actual,
                                   double yaw,
                                   double dt) {
  double r, p, y, t;
  VecX outputs(4);

  // roll, pitch, yaw and throttle (assuming ENU frame)
  r = this->y_controller.calculate(setpoints(1), actual(1), dt);
  p = -this->x_controller.calculate(setpoints(0), actual(0), dt);
  y = yaw;
  t = this->hover_throttle +
      this->z_controller.calculate(setpoints(2), actual(2), dt);
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
  this->setpoint_x = setpoints(0);
  this->setpoint_y = setpoints(1);
  this->setpoint_z = setpoints(2);

  this->output_roll = outputs(0);
  this->output_pitch = outputs(1);
  this->output_throttle = outputs(3);

  return outputs;
}

void PositionController::reset(void) {
  this->x_controller.reset();
  this->y_controller.reset();
  this->z_controller.reset();
}

}  // end of awesomo namespace
