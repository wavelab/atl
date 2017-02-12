#ifndef __AWESOMO_CONTROL_TRACKING_CONTROLLER_HPP__
#define __AWESOMO_CONTROL_TRACKING_CONTROLLER_HPP__

#include <iomanip>

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/pid_controller.hpp"


namespace awesomo {

class TrackingController {
public:
  bool configured;

  double pctrl_dt;
  double vctrl_dt;

  PID x_controller;
  PID y_controller;
  PID z_controller;
  double hover_throttle;

  PID vx_controller;
  PID vy_controller;
  PID vz_controller;

  double roll_limit[2];
  double pitch_limit[2];
  double throttle_limit[2];

  Vec3 pctrl_setpoints;
  Vec4 pctrl_outputs;
  Vec3 vctrl_setpoints;
  Vec4 vctrl_outputs;
  AttitudeCommand att_cmd;

  TrackingController(void);
  int configure(std::string config_file);
  Vec4 calculatePositionErrors(Vec3 errors, double yaw, double dt);
  Vec4 calculateVelocityErrors(Vec3 errors, double yaw, double dt);
  AttitudeCommand calculate(Vec3 pos_errors,
                            Vec3 vel_errors,
                            double yaw,
                            double dt);
  AttitudeCommand calculate(Vec3 target_pos_bf,
                            Vec3 target_vel_bf,
                            Vec3 pos,
                            Vec3 pos_prev,
                            double yaw,
                            double dt);
  void reset(void);
  void printOutputs(void);
  void printErrors(void);
};

}  // end of awesomo namespace
#endif
