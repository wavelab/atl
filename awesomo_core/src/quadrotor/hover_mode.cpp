#include "awesomo_core/quadrotor/hover_mode.hpp"


namespace awesomo {

HoverMode::HoverMode(void) {
  this->configured = false;
  this->hover_height = 0.0;
  this->hover_position << 0.0, 0.0, 0.0;
}

int HoverMode::configure(std::string config_file) {
  ConfigParser parser;

  parser.addParam<double>("hover_height", &this->hover_height);
  parser.addParam<Vec3>("hover_position", &this->hover_position);
  if (parser.load(config_file) != 0) {
    log_err("Failed to configure HoverMode!");
    return -1;
  }

  this->configured = true;
  return 0;
}

void HoverMode::updateHoverPosition(Vec3 pos) {
  this->hover_position = pos;
}

int HoverMode::step(Pose pose, double dt) {
  return 0;
}

}  // end of awesomo namespace
