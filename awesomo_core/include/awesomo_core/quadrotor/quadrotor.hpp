#ifndef __AWESOMO_QUADROTOR_QUADROTOR_HPP__
#define __AWESOMO_QUADROTOR_QUADROTOR_HPP__

#include <iostream>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/control.hpp"
#include "awesomo_core/estimation/estimation.hpp"
#include "awesomo_core/quadrotor/landing_target.hpp"

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
  NOT_SET = -1,
  DISARM_MODE = 0,
  HOVER_MODE = 1,
  DISCOVER_MODE = 2,
  TRACKING_MODE = 3,
  LANDING_MODE = 4
};

class Quadrotor {
public:
  bool configured;

  PositionController position_controller;
  TrackingController tracking_controller;
  LandingController landing_controller;
  AttitudeCommand att_cmd;

  double recover_height;
  bool auto_track;
  bool auto_land;
  bool auto_disarm;
  double target_lost_threshold;
  double min_discover_time;
  double min_tracking_time;

  struct timespec discover_tic;
  struct timespec tracking_tic;
  struct timespec landing_tic;

  enum Mode current_mode;
  double heading;
  Pose pose;
  Vec3 velocity;
  Vec3 hover_position;
  LandingTarget landing_target;
  LandingTarget landing_target_prev;

  Quadrotor(void);
  int configure(std::string config_path);
  int setMode(enum Mode mode);
  int setPose(Pose pose);
  int setVelocity(Vec3 velocity);
  int setHeading(double heading);
  int setTargetPosition(Vec3 position);
  int setTargetVelocity(Vec3 velocity);
  int setTargetDetected(bool detected);
  int setHoverXYPosition(Vec3 position);
  int setHoverPosition(Vec3 position);
  bool conditionsMet(bool *conditions, int nb_conditions);
  int stepHoverMode(double dt);
  int stepDiscoverMode(double dt);
  int stepTrackingMode(double dt);
  int stepLandingMode(double dt);
  int step(double dt);
};

}  // end of awesomo namespace
#endif
