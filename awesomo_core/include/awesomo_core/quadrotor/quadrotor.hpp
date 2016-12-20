#ifndef __QUADROTOR_HPP__
#define __QUADROTOR_HPP__

#include <iostream>

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/control.hpp"
#include "awesomo_core/estimation/estimation.hpp"

namespace awesomo {

enum Mode {
  OFF_MODE = 0,
  HOVER_MODE = 1,
  DISCOVER_MODE = 2,
  TRACKING_MODE = 3,
  LANDING_MODE = 4
};

class Quadrotor {
public:
  bool configured;
  enum Mode mode;

  PositionController position_controller;

  Quadrotor(void);
  int configure(std::string config_path);
  int configurePositionController(std::string config_file);
};

}  // end of awesomo namespace
#endif
