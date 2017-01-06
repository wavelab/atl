#include "awesomo_core/control/tracking_controller.hpp"


namespace awesomo {

int TrackingController::configure(std::string config_file) {
  return PositionController::configure(config_file);
}

}  // end of awesomo namespace
