#ifndef __AWESOMO_QUADROTOR_MODES_BASE_MODE_HPP__
#define __AWESOMO_QUADROTOR_MODES_BASE_MODE_HPP__

#include <time.h>

#include "awesomo_core/utils/utils.hpp"

namespace awesomo {

class BaseMode {
public:
  bool is_running;
  struct timespec time_start;
  struct timespec time_last;

  BaseMode(void);
  virtual void start(void);
  virtual void stop(void);
  virtual void update(void);
  double elasped(void);
};

}  // end of awesomo namespace
#endif
