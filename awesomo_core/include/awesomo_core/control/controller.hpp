#ifndef __CONTROLLER_HPP__
#define __CONTROLLER_HPP__

#include <iostream>
#include <deque>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"

namespace awesomo {

// CONSTANTS
// #define PID_CONFIG
// "/home/chutsu/Dropbox/proj/awesomo/configs/position_controller/pid.yaml"
#define PID_CONFIG \
  "/home/odroid/catkin_ws/src/awesomo/configs/position_controller/pid.yaml"
// #define PID_CONFIG
// "/home/odroid/catkin_ws/src/awesomo/configs/position_controller/pid_semi_ok.yaml"


// STRUCTURES
struct pid {
  int sample_rate;

  float setpoint;
  float output;

  float prev_error;
  float sum_error;

  float p_error;
  float i_error;
  float d_error;

  float k_p;
  float k_i;
  float k_d;

  float dead_zone;
  float min;
  float max;
};

class PositionController {
public:
  struct pid x;
  struct pid y;
  struct pid T;

  float roll;
  float pitch;
  float throttle;
  float hover_throttle;
  Eigen::Quaterniond command_quat;

  PositionController(const std::string config_file);
  void loadConfig(const std::string config_file);
  void calculate(Eigen::Vector3d setpoint,
                 Pose robot,
                 float yaw_setpoint,
                 float dt);
  void reset(void);
};

}  // end of awesomo namespace
#endif
