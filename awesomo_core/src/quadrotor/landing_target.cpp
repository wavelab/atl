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

void LandingTarget::setTargetPosition(Vec3 position) {
  this->position_bf = position;
}

void LandingTarget::setTargetVelocity(Vec3 velocity) {
  this->velocity_bf = velocity;
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

}  // end of awesomo namespace
