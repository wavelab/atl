#ifndef __QUADROTOR_HPP__
#define __QUADROTOR_HPP__

#include <iostream>

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/control.hpp"
#include "awesomo_core/estimation/estimation.hpp"
#include "awesomo_core/quadrotor/base_mode.hpp"
#include "awesomo_core/quadrotor/hover_mode.hpp"
#include "awesomo_core/quadrotor/tracking_mode.hpp"
#include "awesomo_core/quadrotor/landing_mode.hpp"

namespace awesomo {

#define EINVMODE "Invalid quadrotor mode!"
#define FCONFQUAD "Failed to configure quadrotor!"
#define FCONFPCTRL "Failed to configure position controller!"
#define FCONFHMODE "Failed to configure hover mode!"

#define CONFIGURE_CONTROLLER(X, CONF_FILE, ERR_MSG) \
  if (X.configure(CONF_FILE) != 0) {                \
    log_err(ERR_MSG);                               \
    goto error;                                     \
  }

#define CONFIGURE_MODE(X, CONF_FILE, ERR_MSG) \
  if (X.configure(CONF_FILE) != 0) {          \
    log_err(ERR_MSG);                         \
    goto error;                               \
  }

enum Mode {
  DISARM_MODE = 0,
  HOVER_MODE = 1,
  TRACKING_MODE = 2,
  LANDING_MODE = 3
};

class Quadrotor {
public:
  bool configured;

  enum Mode current_mode;
  HoverMode hover_mode;

  PositionController position_controller;

  Quadrotor(void);
  int configure(std::string config_path);
};

}  // end of awesomo namespace
#endif
