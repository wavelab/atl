#include "awesomo_core/quadrotor/modes/tracking_mode.hpp"


namespace awesomo {

TrackingMode::TrackingMode(void) {
  this->configured = false;
  this->prev_mode = DISCOVER_MODE;
  this->next_mode = LANDING_MODE;

  this->min_track_time = 0.0;
  this->target_lost_threshold = 0.0;

  this->target_losted = true;
  this->target_detected = false;
  this->target_bpf = Vec3();
}

int TrackingMode::configure(std::string config_file) {
  ConfigParser parser;

  // clang-format off
  parser.addParam<double>("min_track_time", &this->min_track_time);
  parser.addParam<double>("target_lost_threshold", &this->target_lost_threshold);
  // clang-format on

  if (parser.load(config_file) != 0) {
    log_err("Failed to configure HoverMode!");
    return -1;
  }

  this->configured = true;
  return 0;
}

void TrackingMode::updateTargetPosition(Vec3 position, bool detected) {
  this->target_detected = detected;
  this->target_bpf = position;

  if (detected) {
    this->target_losted = false;
    tic(&this->target_last_seen);
  }
}

void TrackingMode::stop(void) {
  BaseMode::stop();

  // reset target data
  this->target_losted = true;
  this->target_detected = false;
  this->target_bpf = Vec3();
}

void TrackingMode::update(void) {
  BaseMode::update();

  // check target
  if (mtoc(&this->target_last_seen) > this->target_lost_threshold) {
    this->target_losted = true;
  }
}

bool TrackingMode::transition(enum Mode &new_mode) {
  bool min_time_met;

  // setup
  min_time_met = (this->elasped() / 1000.0) > this->min_track_time;

  // transition
  if (min_time_met && this->target_losted == false) {
    new_mode = this->next_mode;
    return true;
  }

  return false;
}

}  // end of awesomo namespace
