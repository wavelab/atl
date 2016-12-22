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

}  // end of awesomo namespace
