#include "awesomo_core/quadrotor/modes/discover_mode.hpp"


namespace awesomo {

DiscoverMode::DiscoverMode(void) {
  this->configured = false;

  this->min_discover_time = 0.0;
  this->target_lost_threshold = 0.0;

  this->target_losted = true;
  this->target_detected = false;
  this->target_bpf = Vec3();
}

int DiscoverMode::configure(std::string config_file) {
  ConfigParser parser;

  // clang-format off
  parser.addParam<double>("min_discover_time", &this->min_discover_time);
  parser.addParam<double>("target_lost_threshold", &this->target_lost_threshold);
  // clang-format on

  if (parser.load(config_file) != 0) {
    log_err("Failed to configure HoverMode!");
    return -1;
  }

  this->configured = true;
  return 0;
}

void DiscoverMode::updateTargetPosition(Vec3 position, bool detected) {
  this->target_detected = detected;
  this->target_bpf = position;

  if (detected) {
    this->target_losted = false;
    tic(&this->target_last_seen);
  }
}

void DiscoverMode::stop(void) {
  BaseMode::stop();

  // reset target data
  this->target_losted = true;
  this->target_detected = false;
  this->target_bpf = Vec3();
}

void DiscoverMode::update(void) {
  BaseMode::update();

  // check target
  if (mtoc(&this->target_last_seen) > this->target_lost_threshold) {
    this->stop();
  }
}

bool DiscoverMode::transition(void) {
  if ((this->elasped() / 1000.0) > this->min_discover_time) {
    return true;
  }

  return false;
}

}  // end of awesomo namespace
