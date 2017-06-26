#ifndef ATL_CONTROL_TRACKING_CONTROLLER_HPP
#define ATL_CONTROL_TRACKING_CONTROLLER_HPP

#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "atl/utils/utils.hpp"
#include "atl/control/pid_controller.hpp"


namespace atl {

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

  TrackingController(void);
  int configure(std::string config_file);
  AttitudeCommand calculate(Vec3 errors, double yaw, double dt);
  AttitudeCommand calculate(Vec3 target_pos_bf,
                            Vec3 pos,
                            Vec3 pos_prev,
                            double yaw,
                            double dt);
  void reset(void);
  void printOutputs(void);
  void printErrors(void);
};

}  // end of atl namespace
#endif
