#ifndef ATL_CONTROL_LANDING_CONTROLLER_HPP
#define ATL_CONTROL_LANDING_CONTROLLER_HPP

#include <deque>
#include <iomanip>
#include <libgen.h>
#include <string>

#include <yaml-cpp/yaml.h>

#include "atl/utils/utils.hpp"
#include "atl/data/data.hpp"
#include "atl/control/pid.hpp"
#include "atl/control/trajectory.hpp"
#include "atl/control/trajectory_index.hpp"

namespace atl {

class LandingController {
public:
  bool configured = false;
  std::string mode;

  double dt = 0.0;
  double blackbox_dt = 0.0;

  double vx_error_prev = 0.0;
  double vy_error_prev = 0.0;
  double vz_error_prev = 0.0;

  double vx_error_sum = 0.0;

  double vx_k_p = 0.0;
  double vx_k_i = 0.0;
  double vx_k_d = 0.0;

  double vy_k_p = 0.0;
  double vy_k_i = 0.0;
  double vy_k_d = 0.0;

  double vz_k_p = 0.0;
  double vz_k_i = 0.0;
  double vz_k_d = 0.0;

  double roll_limit[2] = {0.0, 0.0};
  double pitch_limit[2] = {0.0, 0.0};
  double throttle_limit[2] = {0.0, 0.0};

  Vec3 setpoints{0.0, 0.0, 0.0};
  Vec4 outputs{0.0, 0.0, 0.0, 0.0};
  AttitudeCommand att_cmd;

  TrajectoryIndex traj_index;
  Vec3 trajectory_threshold{1.0, 1.0, 1.0};
  Trajectory trajectory;

  bool blackbox_enable = false;
  double blackbox_rate = FLT_MAX;
  std::ofstream blackbox;

  LandingController() {}

  ~LandingController() {
    if (this->blackbox_enable && this->blackbox) {
      this->blackbox.close();
    }
  }

  /**
   * Configure
   *
   * @param config_file Path to config file
   * @return 0 for success, -1 for failure
   */
  int configure(const std::string &config_file);

  /**
   * Load trajectory
   *
   * @param pos Robot position in inertial frame
   * @param target_pos_bf Target position in body frame
   * @param v Robot velocity in inertial frame
   *
   * @return 0 for success, -1 for failure
   */
  int loadTrajectory(const Vec3 &pos,
                     const Vec3 &target_pos_bf,
                     const double v);

  /**
   * Prepare blackbox
   *
   * @param blackbox_file Path to save blackbox
   *
   * @return 0 for success, -1 for failure
   */
  int prepBlackbox(const std::string &blackbox_file);

  /**
   * Record trajectory index
   *
   * @return 0 for success, -1 for failure
   */
  int recordTrajectoryIndex();

  /**
   * Record landing data
   *
   * @param pos Robot position
   * @param vel Robot velocity
   * @param wp_pos Waypoint position
   * @param wp_vel Waypoint velocity
   * @param wp_inputs Waypoint inputs
   * @param target_pos_bf Target position in body frame
   * @param target_vel_bf Target velocity in body frame
   * @param rpy Roll, pitch, yaw
   * @param thrust Relative thrust (0, 1.0)
   * @param dt Time diffrence in seconds
   *
   * @return 0 for success, -1 for failure
   */
  int record(const Vec3 &pos,
             const Vec3 &vel,
             const Vec2 &wp_pos,
             const Vec2 &wp_vel,
             const Vec2 &wp_inputs,
             const Vec3 &target_pos_bf,
             const Vec3 &target_vel_bf,
             const Vec3 &rpy,
             const double thrust,
             const double dt);

  /**
   * Calculate elocity errors
   *
   * @param v_errors Velocity errors in body frame
   * @param p_errors Position errors in body frame
   * @param yaw Actual robot yaw
   * @param dt Time difference in seconds
   *
   * @return
   *    Attitude command as a vector of size 4:
   *    (roll, pitch, yaw, throttle)
   */
  Vec4 calculateVelocityErrors(const Vec3 &v_errors,
                               const Vec3 &p_errors,
                               const double yaw,
                               const double dt);

  /**
   * Calculate controller outputs
   *
   * @param target_pos_bf Target position in body frame
   * @param target_vel_bf Target velocity in body frame
   * @param pos Actual robot position
   * @param vel Actual robot velocity
   * @param yaw Actual robot yaw
   * @param dt Time difference in seconds
   *
   * @return 0 for success, -1 for failure
   */
  int update(const Vec3 &target_pos_bf,
             const Vec3 &target_vel_bf,
             const Vec3 &pos,
             const Vec3 &vel,
             const Quaternion &orientation,
             const double yaw,
             const double dt);

  /**
   * Reset controller errors to 0
   */
  void reset();

  /**
   * Print controller outputs
   */
  void printOutputs();
};

} // namespace atl
#endif
