#include "awesomo_core/quadrotor/landing_target.hpp"


namespace awesomo {

LandingTarget::LandingTarget(void) {
  this->position_bf << 0.0, 0.0, 0.0;
  this->velocity_bf << 0.0, 0.0, 0.0;
  this->detected = false;
  this->losted = true;
  this->first_seen = (struct timespec){0};
  this->last_seen = (struct timespec){0};

  this->lost_threshold = 1000.0;
}

bool LandingTarget::isTargetLosted(void) {
  if (mtoc(&this->last_seen) > lost_threshold) {
    this->reset();
    return true;
  }

  return false;
}

void LandingTarget::setTarget(Vec3 position, Vec3 velocity, bool detected) {
  // update target position and detected
  this->position_bf = position;
  this->velocity_bf = velocity;
  this->detected = detected;

  // initialize target first seen
  if (this->first_seen.tv_sec == 0) {
    tic(&this->first_seen);
  }

  // update target last seen
  if (detected) {
    this->losted = false;
    tic(&this->last_seen);
  }
}

double LandingTarget::tracked(void) {
  return mtoc(&this->first_seen);
}

void LandingTarget::reset(void) {
  this->position_bf << 0.0, 0.0, 0.0;
  this->velocity_bf << 0.0, 0.0, 0.0;
  this->detected = false;
  this->losted = true;
  this->first_seen = (struct timespec){0};
  this->last_seen = (struct timespec){0};
}

void LandingTarget::update(void) {
  this->isTargetLosted();
}

}  // end of awesomo namespace
