#include "awesomo_core/quadrotor/modes/hover_mode.hpp"


namespace awesomo {

HoverMode::HoverMode(void) {
  this->configured = false;
  this->hover_position << 0.0, 0.0;
  this->hover_height = 0.0;
}

int HoverMode::configure(std::string config_file) {
  ConfigParser parser;
  Vec3 position;

  parser.addParam<Vec3>("hover_position", &position);
  if (parser.load(config_file) != 0) {
    log_err("Failed to configure HoverMode!");
    return -1;
  }
  this->hover_position << position(0), position(1);
  this->hover_height = position(2);

  this->configured = true;
  return 0;
}

void HoverMode::updateHoverXYPosition(double x, double y) {
  this->hover_position(0) = x;
  this->hover_position(1) = y;
}

void HoverMode::updateHoverPosition(Vec3 pos) {
  this->hover_position(0) = pos(0);
  this->hover_position(1) = pos(1);
  this->hover_height = pos(2);
}

Vec3 HoverMode::getHoverPosition(void) {
  Vec3 position;

  position(0) = this->hover_position(0);
  position(1) = this->hover_position(1);
  position(2) = this->hover_height;

  return position;
}

}  // end of awesomo namespace
