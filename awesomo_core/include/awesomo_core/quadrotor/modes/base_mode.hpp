#ifndef __AWESOMO_QUADROTOR_MODES_BASE_MODE_HPP__
#define __AWESOMO_QUADROTOR_MODES_BASE_MODE_HPP__

#include <time.h>

#include "awesomo_core/utils/utils.hpp"

namespace awesomo {

enum Mode {
  NOT_SET = -1,
  DISARM_MODE = 0,
  HOVER_MODE = 1,
  DISCOVER_MODE = 2,
  TRACKING_MODE = 3,
  LANDING_MODE = 4
};

class BaseMode {
public:
  bool is_running;
  struct timespec time_start;
  struct timespec time_last;
  enum Mode prev_mode;
  enum Mode next_mode;

  BaseMode(void);
  virtual void start(void);
  virtual void stop(void);
  virtual void update(void);
  double elasped(void);
};

}  // end of awesomo namespace
#endif
