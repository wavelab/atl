#include "awesomo_core/quadrotor/quadrotor.hpp"


namespace awesomo {

Quadrotor::Quadrotor(void) {
  this->configured = false;
  this->mode = HOVER_MODE;
}

int Quadrotor::configure(std::string config_path) {
  std::string config_file;

  // clang-format off
  config_file = config_path + "/" + "position_controller.yaml";
  if (this->configurePositionController(config_file) != 0) { goto error; };
  // clang-format on

  this->configured = true;
  return 0;

error:
  log_err(FCONFQUAD);
  return -1;
}

int Quadrotor::configurePositionController(std::string config_file) {
  int retval;

  retval = this->position_controller.configure(config_file);
  if (retval != 0) {
    log_err(FCONFPCTRL);
    return -1;
  } else {
    return 0;
  }
}

int Quadrotor::step(void) {
  switch (this->mode) {
    case HOVER_MODE:
      break;
    case TRACKING_MODE:
      break;
    case LANDING_MODE:
      break;
    default:
      log_err(EINVMODE);
      return -1;
  }

  return 0;
}


}  // end of awesomo namespace
