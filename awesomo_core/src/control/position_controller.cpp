#include "awesomo_control/position_controller.hpp"


namespace awesomo {

PositionController::PositionController(void) {
  this->x_controller = PID(0.9, 0.01, 0.33);
  this->y_controller = PID(0.9, 0.01, 0.33);
  this->z_controller = PID(3.0, 0.0, 0.5);

  roll_limit[0] = 0.0;
  roll_limit[1] = 0.0;

  pitch_limit[0] = 0.0;
  pitch_limit[1] = 0.0;
}

void PositionController::loadConfig(const std::string config_file) {
  try {
    YAML::Node config = YAML::LoadFile(config_file);

    // clang-format off
    // roll controller
    this->x_controller = PID(
      config["x_controller"]["k_p"].as<float>(),
      config["x_controller"]["k_i"].as<float>(),
      config["x_controller"]["k_d"].as<float>()
    );
    roll_limit[0] = config["roll_controller"]["min"].as<float>();
    roll_limit[1] = config["roll_controller"]["min"].as<float>();

    // pitch controller
    this->y_controller = PID(
      config["y_controller"]["k_p"].as<float>(),
      config["y_controller"]["k_i"].as<float>(),
      config["y_controller"]["k_d"].as<float>()
    );
    pitch_limit[0] = config["pitch_controller"]["min"].as<float>();
    pitch_limit[1] = config["pitch_controller"]["min"].as<float>();

    // throttle_controller
    this->y_controller = PID(
      config["z_controller"]["k_p"].as<float>(),
      config["z_controller"]["k_i"].as<float>(),
      config["z_controller"]["k_d"].as<float>()
    );
    // clang-format on

  } catch (YAML::BadFile &ex) {
    throw;
  }
}

VecX PositionController::calculate(VecX setpoints,
                                   VecX actual,
                                   double yaw,
                                   double dt) {
  double max_thrust;
  double r, p, y, t;
  VecX outputs(4);

  // roll, pitch, yaw and thrust
  r = -this->y_controller.calculate(setpoints(1), actual(1), dt);
  p = this->x_controller.calculate(setpoints(0), actual(0), dt);
  y = yaw;
  t = 0.5 + this->z_controller.calculate(setpoints(2), actual(2), dt);
  outputs << r, p, y, t;

  // limit roll, pitch and yaw
  for (int i = 0; i < 3; i++) {
    if (outputs(i) > deg2rad(30.0)) {
      outputs(i) = deg2rad(30.0);
    } else if (outputs(i) < deg2rad(-30.0)) {
      outputs(i) = deg2rad(-30.0);
    }
  }

  // limit thrust
  if (outputs(3) > 1.0) {
    outputs(3) = 1.0;
  } else if (outputs(3) < 0.0) {
    outputs(3) = 0.0;
  }

  return outputs;
}

void PositionController::reset(void) {
  this->x_controller.reset();
  this->y_controller.reset();
  this->z_controller.reset();
}

}  // end of awesomo namespace
