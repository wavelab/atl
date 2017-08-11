#ifndef ATL_CONTROL_LANDING_CONTROLLER_HPP
#define ATL_CONTROL_LANDING_CONTROLLER_HPP

#include <libgen.h>
#include <deque>
#include <iomanip>
#include <string>

#include <yaml-cpp/yaml.h>

#include "atl/control/pid.hpp"
#include "atl/control/trajectory.hpp"
#include "atl/control/trajectory_index.hpp"
#include "atl/utils/utils.hpp"

namespace atl {

class LandingController {
public:
  bool configured;
  std::string mode;

  double dt;
  double blackbox_dt;

  double vx_error_prev;
  double vy_error_prev;
  double vz_error_prev;

  double vx_error_sum;

  double vx_k_p;
  double vx_k_i;
  double vx_k_d;

  double vy_k_p;
  double vy_k_i;
  double vy_k_d;

  double vz_k_p;
  double vz_k_i;
  double vz_k_d;

  double roll_limit[2];
  double pitch_limit[2];
  double throttle_limit[2];

  Vec3 setpoints;
  Vec4 outputs;
  AttitudeCommand att_cmd;

  TrajectoryIndex traj_index;
  Vec3 trajectory_threshold;
  Trajectory trajectory;

  bool blackbox_enable;
  double blackbox_rate;
  std::ofstream blackbox;

  LandingController()
      : configured{false},
        dt{0.0},
        blackbox_dt{0.0},
        vx_error_prev{0.0},
        vy_error_prev{0.0},
        vz_error_prev{0.0},
        vx_error_sum{0.0},
        vx_k_p{0.0},
        vx_k_i{0.0},
        vx_k_d{0.0},
        vy_k_p{0.0},
        vy_k_i{0.0},
        vy_k_d{0.0},
        vz_k_p{0.0},
        vz_k_i{0.0},
        vz_k_d{0.0},
        roll_limit{0.0, 0.0},
        pitch_limit{0.0, 0.0},
        throttle_limit{0.0, 0.0},
        setpoints{0.0, 0.0, 0.0},
        outputs{0.0, 0.0, 0.0, 0.0},
        att_cmd{},
        trajectory_threshold{1.0, 1.0, 1.0},
        blackbox_enable{false},
        blackbox_rate{FLT_MAX} {}

  ~LandingController() {
    if (this->blackbox_enable && this->blackbox) {
      this->blackbox.close();
    }
  }

  int configure(const std::string &config_file);

  int loadTrajectory(Vec3 pos, Vec3 target_pos_bf, double v);

  int prepBlackbox(const std::string &blackbox_file);

  int recordTrajectoryIndex();

  int record(
    Vec3 pos,
    Vec3 vel,
    Vec2 wp_pos,
    Vec2 wp_vel,
    Vec2 wp_inputs,
    Vec3 target_pos_bf,
    Vec3 target_vel_bf,
    Vec3 rpy,
    double thrust,
    double dt);

  Vec4 calculateVelocityErrors(
    Vec3 v_errors, Vec3 p_errors, double yaw, double dt);

  int calculate(
    Vec3 target_pos_bf,
    Vec3 target_vel_bf,
    Vec3 pos,
    Vec3 vel,
    Quaternion orientation,
    double yaw,
    double dt);

  void reset();

  void printOutputs();
};

}  // namespace atl
#endif
