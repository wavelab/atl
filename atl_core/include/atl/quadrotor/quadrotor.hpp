#ifndef ATL_QUADROTOR_QUADROTOR_HPP
#define ATL_QUADROTOR_QUADROTOR_HPP

#include <iostream>

#include "atl/control/control.hpp"
#include "atl/estimation/estimation.hpp"
#include "atl/quadrotor/landing_target.hpp"
#include "atl/utils/utils.hpp"

namespace atl {

#define INFO_KMODE "[DISARM_MODE]!"
#define INFO_HMODE "[HOVER_MODE]!"
#define INFO_DMODE "[DISCOVER_MODE]!"
#define INFO_TMODE "[TRACKING_MODE]!"
#define INFO_LMODE "[LANDING_MODE]!"
#define INFO_WMODE "[WAYPOINT_MODE]!"
#define EINVMODE "Invalid quadrotor mode!"
#define FCONFQUAD "Failed to configure quadrotor!"
#define FCONFPCTRL "Failed to configure position controller!"
#define FCONFTCTRL "Failed to configure tracking controller!"
#define FCONFLCTRL "Failed to configure landing controller!"
#define FCONFWCTRL "Failed to configure waypoint controller!"
#define FCONFHMODE "Failed to configure hover mode!"
#define FCONFDMODE "Failed to configure discover mode!"
#define FCONFTMODE "Failed to configure tracking mode!"

#define CONFIGURE_CONTROLLER(X, CONF_FILE, ERR_MSG) \
  if (X.configure(CONF_FILE) != 0) {                \
    LOG_ERROR(ERR_MSG);                             \
    goto error;                                     \
  }

#define CONFIGURE_MODE(X, CONF_FILE, ERR_MSG) \
  if (X.configure(CONF_FILE) != 0) {          \
    LOG_ERROR(ERR_MSG);                       \
    goto error;                               \
  }

enum Mode {
  NOT_SET = -1,
  DISARM_MODE = 0,
  HOVER_MODE = 1,
  DISCOVER_MODE = 2,
  TRACKING_MODE = 3,
  LANDING_MODE = 4,
  WAYPOINT_MODE = 5
};

class Quadrotor {
public:
  bool configured = false;

  PositionController position_controller;
  TrackingController tracking_controller;
  LandingController landing_controller;
  WaypointController waypoint_controller;
  AttitudeCommand att_cmd;

  double recover_height = 0.0;
  bool auto_track = 0.0;
  bool auto_land = 0.0;
  bool auto_disarm = 0.0;
  double target_lost_threshold = 0.0;
  double min_discover_time = FLT_MAX;
  double min_tracking_time = FLT_MAX;

  struct timespec discover_tic = {0, 0};
  struct timespec tracking_tic = {0, 0};
  struct timespec landing_tic = {0, 0};

  bool home_set = false;
  double home_lat = 0.0;
  double home_lon = 0.0;

  Mission mission;

  enum Mode current_mode = NOT_SET;
  double yaw = 0.0;
  Pose pose;
  Vec3 velocity = Vec3::Zero();
  Vec3 hover_position = Vec3::Zero();
  LandingTarget landing_target;
  LandingTarget landing_target_prev;

  Quadrotor() {}

  /**
   * Configure
   * @param config_path Path to configs
   * @return
   *    - 0: success
   *    - -1: failure to load / parse configuration file
   */
  int configure(const std::string &config_path);

  /**
   * Set home point
   *
   * @param latitude Latitude in decimal format
   * @param longitude Longitude in decimal format
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int setHomePoint(const double latitude, const double longitude);

  /**
   * Set mode
   *
   * @param mode Mode
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int setMode(const enum Mode &mode);

  /**
   * Set pose
   *
   * @param pose Pose
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int setPose(const Pose &pose);

  /**
   * Set velocity
   *
   * @param velocity Velocity
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int setVelocity(const Vec3 &velocity);

  /**
   * Set yaw
   *
   * @param yaw Yaw in radians
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int setYaw(const double yaw);

  /**
   * Set target velocity
   *
   * @param velocity Target velocity
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int setTargetPosition(const Vec3 &position);

  /**
   * Set target velocity
   *
   * @param velocity Target velocity
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int setTargetVelocity(const Vec3 &velocity);

  /**
   * Set target detected
   *
   * @param detected Boolean to denote whether target is observed
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int setTargetDetected(const bool detected);

  /**
   * Set hover x-y position
   *
   * @param position Position
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int setHoverXYPosition(const Vec3 &position);

  /**
   * Set hover position
   *
   * @param position Position
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int setHoverPosition(const Vec3 &position);

  /**
   *  Check whether all conditions have been met
   *
   * @param conditions Conditions to check
   * @param nb_conditions Number of conditions
   * @return
   *    - 0: All conditions have been met
   *    - -1: Not all conditions have been met
   */
  bool conditionsMet(bool *conditions, int nb_conditions);

  /**
   * Step hover mode
   *
   * @param dt Time difference in seconds
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int stepHoverMode(const double dt);

  /**
   * Step discover mode
   *
   * @param dt Time difference in seconds
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int stepDiscoverMode(const double dt);

  /**
   * Step tracking mode
   *
   * @param dt Time difference in seconds
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int stepTrackingMode(const double dt);

  /**
   * Step landing mode
   *
   * @param dt Time difference in seconds
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int stepLandingMode(const double dt);

  /**
   * Step waypoint mode
   *
   * @param dt Time difference in seconds
   * @return
   *    - 0: Success
   *    - -1: Not configured
   *    - -2: Home point not set
   *    - -3: Failed to convert mission to local waypoints
   */
  int stepWaypointMode(const double dt);

  /**
   * Rest quadrotor
   *
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int reset();

  /**
   * Step quadrotor
   *
   * @return
   *    - 0: Success
   *    - -1: Not configured
   */
  int step(const double dt);
};

}  // namespace atl
#endif
