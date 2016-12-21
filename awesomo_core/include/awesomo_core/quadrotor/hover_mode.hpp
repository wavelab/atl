#ifndef __AWESOMO_QUADROTOR_HOVER_MODE_HPP__
#define __AWESOMO_QUADROTOR_HOVER_MODE_HPP__

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/quadrotor/base_mode.hpp"


namespace awesomo {

class HoverMode : public BaseMode {
public:
  bool configured;
  double hover_height;

  HoverMode(void);
  int configure(std::string config_file);
};

}  // end of awesomo namespace
#endif
