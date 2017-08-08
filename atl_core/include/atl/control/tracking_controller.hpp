#ifndef ATL_CONTROL_TRACKING_CONTROLLER_HPP
#define ATL_CONTROL_TRACKING_CONTROLLER_HPP

#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "atl/control/pid.hpp"
#include "atl/utils/utils.hpp"

namespace atl {

/**
 * Tracking Controller
 */
class TrackingController {
public:
  bool configured;

  double dt;
  PID x_controller;
  PID y_controller;
  PID z_controller;

  double hover_throttle;
  double roll_limit[2];
  double pitch_limit[2];
  Vec3 track_offset;

  Vec3 setpoints;
  Vec4 outputs;
  AttitudeCommand att_cmd;

  TrackingController();
  int configure(std::string config_file);
  AttitudeCommand calculate(Vec3 errors, double yaw, double dt);
  AttitudeCommand calculate(
    Vec3 target_pos_bf, Vec3 pos, Vec3 pos_prev, double yaw, double dt);
  void reset();
  void printOutputs();
  void printErrors();
};

}  // namespace atl
#endif
