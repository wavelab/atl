#ifndef __QUADROTOR_HPP__
#define __QUADROTOR_HPP__

#include <iostream>

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/control.hpp"
#include "awesomo_core/estimation/estimation.hpp"

namespace awesomo {

#define EINVMODE "Invalid quadrotor mode!"
#define FCONFQUAD "Failed to configure quadrotor!"
#define FCONFPCTRL "Failed to configure position controller"

enum Mode { HOVER_MODE = 1, TRACKING_MODE = 2, LANDING_MODE = 3 };

class Quadrotor {
public:
  bool configured;
  enum Mode mode;

  PositionController position_controller;

  Quadrotor(void);
  int configure(std::string config_path);
  int configurePositionController(std::string config_file);
  int step(void);
};

}  // end of awesomo namespace
#endif
