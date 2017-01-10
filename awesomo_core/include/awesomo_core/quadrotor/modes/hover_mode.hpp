#ifndef __AWESOMO_QUADROTOR_MODES_HOVER_MODE_HPP__
#define __AWESOMO_QUADROTOR_MODES_HOVER_MODE_HPP__

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/quadrotor/modes/base_mode.hpp"
#include "awesomo_core/control/position_controller.hpp"


namespace awesomo {

class HoverMode : public BaseMode {
public:
  bool configured;
  Vec2 hover_position;
  double hover_height;

  HoverMode(void);
  int configure(std::string config_file);
  void updateHoverXYPosition(double x, double y);
  void updateHoverPosition(Vec3 pos);
  Vec3 getHoverPosition(void);
};

}  // end of awesomo namespace
#endif
