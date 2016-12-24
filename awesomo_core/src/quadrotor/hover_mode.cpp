#include "awesomo_core/quadrotor/hover_mode.hpp"


namespace awesomo {

HoverMode::HoverMode(void) {
  this->configured = false;
  this->hover_height = 0.0;
  this->hover_position << 0.0, 0.0, 0.0;
}

int HoverMode::configure(std::string config_file) {
  YAML::Node config;

  try {
    // pre-check
    if (file_exists(config_file) == false) {
      log_err("File not found: %s", config_file.c_str());
      return -1;
    }

    // parse configs
    config = YAML::LoadFile(config_file);
    this->hover_height = yamlDoubleToDouble(config["hover_height"]);
    this->hover_position = yamlVec3ToVec3(config["hover_position"]);

  } catch (std::exception &ex) {
    std::cout << ex.what();
    return -2;
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
