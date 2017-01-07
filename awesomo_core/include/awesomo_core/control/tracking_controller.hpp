#ifndef __AWESOMO_CONTROL_TRACKING_CONTROLLER_HPP__
#define __AWESOMO_CONTROL_TRACKING_CONTROLLER_HPP__

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/pid_controller.hpp"
#include "awesomo_core/control/position_controller.hpp"


namespace awesomo {

class TrackingController {
public:
  TrackingController(void);
  int configure(std::string config_file);
};

}  // end of awesomo namespace
#endif
