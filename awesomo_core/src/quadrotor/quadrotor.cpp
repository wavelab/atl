#include "awesomo_core/quadrotor/quadrotor.hpp"


namespace awesomo {

Quadrotor::Quadrotor(void) {
  this->configured = false;
  this->mode = OFF_MODE;
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
  log_err("failed to configure quadrotor!");
  return -1;
}

int Quadrotor::configurePositionController(std::string config_file) {
  int retval;

  retval = this->position_controller.configure(config_file);
  if (retval != 0) {
    log_err("failed to configure position controller");
    return -1;
  } else {
    return 0;
  }
}


}  // end of awesomo namespace
