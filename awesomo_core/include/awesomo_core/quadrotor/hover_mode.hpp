#ifndef __AWESOMO_QUADROTOR_HOVER_MODE_HPP__
#define __AWESOMO_QUADROTOR_HOVER_MODE_HPP__

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/quadrotor/base_mode.hpp"
#include "awesomo_core/control/position_controller.hpp"


namespace awesomo {

class HoverMode : public BaseMode {
public:
  bool configured;
  double hover_height;
  Vec3 hover_position;

  HoverMode(void);
  int configure(std::string config_file);
  void updateHoverPosition(Vec3 pos);
  int step(Pose pose, double dt);
};

}  // end of awesomo namespace
#endif
