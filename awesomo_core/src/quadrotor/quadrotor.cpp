#include "awesomo_core/quadrotor/quadrotor.hpp"


namespace awesomo {

Quadrotor::Quadrotor(void) {
  this->configured = false;

  this->current_mode = HOVER_MODE;
}

int Quadrotor::configure(std::string config_path) {
  std::string config_file;

  // position controller
  config_file = config_path + "/controllers/" + "position_controller.yaml";
  CONFIGURE_CONTROLLER(this->position_controller, config_file, FCONFPCTRL);

  // hover mode
  config_file = config_path + "/modes/" + "hover_mode.yaml";
  CONFIGURE_MODE(this->hover_mode, config_file, FCONFHMODE);

  this->current_mode = HOVER_MODE;
  this->configured = true;

  return 0;
error:
  return -1;
}

int Quadrotor::stepHoverMode(Pose pose, double dt) {
  Vec3 actual;
  Vec3 setpoint;
  Vec4 output;

  setpoint = this->hover_mode.hover_position;
  actual = pose.position;

  // clang-format off
  output = this->position_controller.calculate(
    setpoint,
    actual,
    this->heading,
    dt
  );
  this->att_cmd = AttitudeCommand(output);
  // clang-format on

  return 0;
}

int Quadrotor::step(Pose pose, double dt) {
  switch (this->current_mode) {
    case HOVER_MODE:
      this->stepHoverMode(pose, dt);
      break;

    case TRACKING_MODE:
      this->hover_mode.updateHoverPosition(pose.position);
      break;

    case LANDING_MODE:
      this->hover_mode.updateHoverPosition(pose.position);
      break;

    case DISARM_MODE:
      return -1;

    default:
      log_err(EINVMODE);
      return -1;
  }

  return 0;
}

}  // end of awesomo namespace
