#ifndef __AWESOMO_QUADROTOR_MODES_DISCOVER_MODE_HPP__
#define __AWESOMO_QUADROTOR_MODES_DISCOVER_MODE_HPP__

#include "awesomo_core/quadrotor/modes/base_mode.hpp"


namespace awesomo {

class DiscoverMode : public BaseMode {
public:
  bool configured;

  double min_discover_time;
  double target_lost_threshold;

  bool target_losted;
  bool target_detected;
  Vec3 target_bpf;
  struct timespec target_last_seen;

  DiscoverMode(void);
  int configure(std::string config_file);
  void updateTargetPosition(Vec3 position, bool detected);
  void stop(void);
  void update(void);
  bool transition(void);
};

}  // end of awesomo namespace
#endif
