#include "atl/data/landing_target.hpp"

namespace atl {

bool LandingTarget::isTargetLosted() {
  if (mtoc(&this->last_seen) > lost_threshold) {
    this->reset();
    return true;
  }

  return false;
}

void LandingTarget::setTargetPosition(Vec3 position) {
  this->position_B = position;
}

void LandingTarget::setTargetVelocity(Vec3 velocity) {
  this->velocity_B = velocity;
}

double LandingTarget::tracked() { return mtoc(&this->first_seen); }

void LandingTarget::reset() {
  this->position_B << 0.0, 0.0, 0.0;
  this->velocity_B << 0.0, 0.0, 0.0;
  this->detected = false;
  this->losted = true;
  this->first_seen = (struct timespec){0, 0};
  this->last_seen = (struct timespec){0, 0};
}

void LandingTarget::update(bool detected) {
  // initialize target first seen
  if (this->first_seen.tv_sec == 0) {
    tic(&this->first_seen);
  }

  // update target last seen
  this->detected = detected;
  if (detected) {
    this->losted = false;
    tic(&this->last_seen);
  }

  // target losted?
  this->isTargetLosted();
}

} // namespace atl
