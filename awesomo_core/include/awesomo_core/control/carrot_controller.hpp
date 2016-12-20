#ifndef __AWESOMO_CONTROL_CARROT_CONTROLLER_HPP__
#define __AWESOMO_CONTROL_CARROT_CONTROLLER_HPP__

#include "awesomo_core/utils/utils.hpp"


namespace awesomo {

class CarrotController {
public:
  bool configured;

  std::deque<Vec3> waypoints;

  double look_ahead_dist;
  double wp_threshold;
  Vec3 wp_start;
  Vec3 wp_end;
  Vec3 carrot_prev;

  CarrotController(void);
  int configure(std::string config_file_path);
  int configure(std::deque<Vec3> wps,
                double look_ahead_dist,
                double wp_threshold);
  Vec3 closestPoint(Vec3 position, Vec3 wp_start, Vec3 wp_end);
  Vec3 calculate(Vec3 position, double r, Vec3 wp_start, Vec3 wp_end);
  int waypointReached(Vec3 position, Vec3 waypoint, double threshold);
  int update(Vec3 position, Vec3 &carrot);
};

}  // end of awesomo namespace
#endif
