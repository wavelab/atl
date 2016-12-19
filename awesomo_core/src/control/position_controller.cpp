#include "awesomo_control/position_controller.hpp"


namespace awesomo {

PositionController::PositionController(void) {
  this->x_controller = PID(0.9, 0.01, 0.33);
  this->y_controller = PID(0.9, 0.01, 0.33);
  this->z_controller = PID(3.0, 0.0, 0.5);
}

void PositionController::loadConfig(const std::string config_file) {
  try {
    YAML::Node config = YAML::LoadFile(config_file);

    // roll controller
    this->x.setpoint = config["roll_controller"]["setpoint"].as<float>();
    this->x.output = 0.0f;
    this->x.prev_error = 0.0f;
    this->x.sum_error = 0.0f;
    this->x.p_error = 0.0f;
    this->x.i_error = 0.0f;
    this->x.d_error = 0.0f;
    this->x.k_p = config["roll_controller"]["k_p"].as<float>();
    this->x.k_i = config["roll_controller"]["k_i"].as<float>();
    this->x.k_d = config["roll_controller"]["k_d"].as<float>();
    this->x.dead_zone = config["roll_controller"]["deadzone"].as<float>();
    this->x.min = config["roll_controller"]["min"].as<float>();
    this->x.max = config["roll_controller"]["max"].as<float>();

    // pitch controller
    this->y.setpoint = config["roll_controller"]["setpoint"].as<float>();
    this->y.output = 0.0f;
    this->y.prev_error = 0.0f;
    this->y.sum_error = 0.0f;
    this->y.p_error = 0.0f;
    this->y.i_error = 0.0f;
    this->y.d_error = 0.0f;
    this->y.k_p = config["roll_controller"]["k_p"].as<float>();
    this->y.k_i = config["roll_controller"]["k_i"].as<float>();
    this->y.k_d = config["roll_controller"]["k_d"].as<float>();
    this->y.dead_zone = config["roll_controller"]["deadzone"].as<float>();
    this->y.min = config["roll_controller"]["min"].as<float>();
    this->y.max = config["roll_controller"]["max"].as<float>();

    // throttle_controller
    this->hover_throttle =
      config["throttle_controller"]["hover_throttle"].as<float>();
    this->T.setpoint = config["throttle_controller"]["setpoint"].as<float>();
    this->T.output = 0.0f;
    this->T.prev_error = 0.0f;
    this->T.sum_error = 0.0f;
    this->T.p_error = 0.0f;
    this->T.i_error = 0.0f;
    this->T.d_error = 0.0f;
    this->T.k_p = config["throttle_controller"]["k_p"].as<float>();
    this->T.k_i = config["throttle_controller"]["k_i"].as<float>();
    this->T.k_d = config["throttle_controller"]["k_d"].as<float>();
    this->T.dead_zone = config["throttle_controller"]["deadzone"].as<float>();
    this->T.min = config["throttle_controller"]["min"].as<float>();
    this->T.max = config["throttle_controller"]["max"].as<float>();

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
  this->roll = 0;
  this->pitch = 0;
  this->throttle = 0;

  this->x.output = 0.0f;
  this->x.prev_error = 0.0f;
  this->x.sum_error = 0.0f;
  this->x.p_error = 0.0f;
  this->x.i_error = 0.0f;
  this->x.d_error = 0.0f;

  this->y.output = 0.0f;
  this->y.prev_error = 0.0f;
  this->y.sum_error = 0.0f;
  this->y.p_error = 0.0f;
  this->y.i_error = 0.0f;
  this->y.d_error = 0.0f;

  this->T.output = 0.0f;
  this->T.prev_error = 0.0f;
  this->T.sum_error = 0.0f;
  this->T.p_error = 0.0f;
  this->T.i_error = 0.0f;
  this->T.d_error = 0.0f;
}

}  // end of awesomo namespace
