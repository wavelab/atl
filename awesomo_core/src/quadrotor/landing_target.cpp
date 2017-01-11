#include "awesomo_core/quadrotor/landing_target.hpp"


namespace awesomo {

LandingTarget::LandingTarget(void) {
  this->target_bpf << 0.0, 0.0, 0.0;
  this->target_detected = false;
  this->target_losted = true;
  this->first_seen = (struct timespec){0};
  this->last_seen = (struct timespec){0};

  this->lost_threshold = 1000.0;
}

bool LandingTarget::isTargetLosted(void) {
  if (mtoc(&this->last_seen) > lost_threshold) {
    log_info("Landing Target is losted!");
    this->reset();
    return true;
  }

  return false;
}

void LandingTarget::setTargetPosition(Vec3 position, bool detected) {
  // update target position and detected
  this->target_bpf = position;
  this->target_detected = detected;

  // initialize target first seen
  if (this->first_seen.tv_sec == 0) {
    tic(&this->first_seen);
  }

  // update target last seen
  if (detected) {
    this->target_losted = false;
    tic(&this->last_seen);
  }
}

double LandingTarget::tracked(void) {
  return mtoc(&this->first_seen);
}

void LandingTarget::reset(void) {
  this->target_bpf << 0.0, 0.0, 0.0;
  this->target_detected = false;
  this->target_losted = true;
  this->first_seen = (struct timespec){0};
  this->last_seen = (struct timespec){0};
}

void LandingTarget::update(void) {
  this->isTargetLosted();
}

}  // end of awesomo namespace
