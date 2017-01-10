#ifndef __QUADROTOR_HPP__
#define __QUADROTOR_HPP__

#include <iostream>

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/control.hpp"
#include "awesomo_core/estimation/estimation.hpp"
#include "awesomo_core/quadrotor/modes/base_mode.hpp"
#include "awesomo_core/quadrotor/modes/discover_mode.hpp"
#include "awesomo_core/quadrotor/modes/hover_mode.hpp"
#include "awesomo_core/quadrotor/modes/tracking_mode.hpp"
#include "awesomo_core/quadrotor/modes/landing_mode.hpp"

namespace awesomo {

#define INFO_KMODE "[DISARM_MODE]!"
#define INFO_HMODE "[HOVER_MODE]!"
#define INFO_DMODE "[DISCOVER_MODE]!"
#define INFO_TMODE "[TRACKING_MODE]!"
#define INFO_LMODE "[LANDING_MODE]!"
#define EINVMODE "Invalid quadrotor mode!"
#define FCONFQUAD "Failed to configure quadrotor!"
#define FCONFPCTRL "Failed to configure position controller!"
#define FCONFTCTRL "Failed to configure tracking controller!"
#define FCONFHMODE "Failed to configure hover mode!"
#define FCONFDMODE "Failed to configure discover mode!"
#define FCONFTMODE "Failed to configure tracking mode!"

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
  DISCOVER_MODE = 2,
  TRACKING_MODE = 3,
  LANDING_MODE = 4
};

class Quadrotor {
public:
  bool configured;

  enum Mode current_mode;
  HoverMode hover_mode;
  DiscoverMode discover_mode;
  TrackingMode tracking_mode;

  double heading;
  Pose pose;

  PositionController position_controller;
  TrackingController tracking_controller;
  AttitudeCommand att_cmd;

  Quadrotor(void);
  int configure(std::string config_path);
  void setMode(enum Mode mode);
  void setPose(Pose pose);
  void setTargetPosition(Vec3 position, bool detected);
  int stepHoverMode(double dt);
  int stepDiscoverMode(double dt);
  int stepTrackingMode(double dt);
  int step(double dt);
};

}  // end of awesomo namespace
#endif
