#include "awesomo_core/quadrotor/base_mode.hpp"


namespace awesomo {

BaseMode::BaseMode(void) {
  this->is_running = false;
  this->time_start = (struct timespec){0};
  this->time_last = (struct timespec){0};
}

void BaseMode::start(void) {
  this->is_running = true;
  tic(&this->time_start);
  tic(&this->time_last);
}

void BaseMode::stop(void) {
  this->is_running = false;
  this->time_start = (struct timespec){0};
  this->time_last = (struct timespec){0};
}

void BaseMode::update(void) {
  tic(&this->time_last);
}

double BaseMode::duration(void) {
  mtoc(&this->time_start);
}

}  // end of awesomo namespace
