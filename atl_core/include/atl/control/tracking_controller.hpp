#ifndef ATL_CONTROL_TRACKING_CONTROLLER_HPP
#define ATL_CONTROL_TRACKING_CONTROLLER_HPP

#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "atl/control/pid.hpp"
#include "atl/data/data.hpp"
#include "atl/utils/utils.hpp"

namespace atl {

/**
 * Tracking Controller
 */
class TrackingController {
public:
  bool configured = false;

  double dt = 0.0;
  PID x_controller;
  PID y_controller;
  PID z_controller;

  double roll_limit[2] = {0.0, 0.0};
  double pitch_limit[2] = {0.0, 0.0};
  double hover_throttle = 0.0;
  Vec3 track_offset{0.0, 0.0, 0.0};

  Vec3 setpoints{0.0, 0.0, 0.0};
  Vec4 outputs{0.0, 0.0, 0.0, 0.0};

  TrackingController() {}

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @return 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Update controller
   *
   * @param errors_B Tracking errors in body frame
   * @param yaw_W Actual yaw in world frame
   * @param dt Time difference in seconds
   *
   * @return controller output
   */
  AttitudeCommand update(const Vec3 &errors_B,
                         const double yaw_W,
                         const double dt);

  /**
   * Update controller
   *
   * @param target_pos_B Target position in body frame
   * @param pos Position in world frame
   * @param pos_prev Previous position in world frame
   * @param yaw Actual yaw in world frame
   * @param dt Time difference in seconds
   *
   * @return controller output
   */
  AttitudeCommand update(const Vec3 &target_pos_B,
                         const Vec3 &pos_W,
                         const Vec3 &pos_prev_W,
                         const double yaw_W,
                         const double dt);

  /**
   * Reset controller errors to 0
   */
  void reset();

  /**
   * Print controller outputs
   */
  void printOutputs();

  /**
   * Print controller errors
   */
  void printErrors();
};

} // namespace atl
#endif
