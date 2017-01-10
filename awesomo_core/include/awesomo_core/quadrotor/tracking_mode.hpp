#ifndef __AWESOMO_QUADROTOR_TRACKING_MODE_HPP__
#define __AWESOMO_QUADROTOR_TRACKING_MODE_HPP__

#include "awesomo_core/quadrotor/base_mode.hpp"


namespace awesomo {

class TrackingMode : public BaseMode {
public:
  bool configured;

  double min_track_time;
  double target_lost_threshold;

  bool target_losted;
  bool target_detected;
  Vec3 target_bpf;
  struct timespec target_last_seen;

  TrackingMode(void);
  int configure(std::string config_file);
  void updateTargetPosition(Vec3 position, bool detected);
  void update(void);
};

}  // end of awesomo namespace
#endif
