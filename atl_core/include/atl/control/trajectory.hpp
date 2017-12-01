#ifndef ATL_CONTROL_TRAJECTORY_HPP
#define ATL_CONTROL_TRAJECTORY_HPP

#include <deque>
#include <iomanip>
#include <libgen.h>
#include <string>

#include <yaml-cpp/yaml.h>

#include "atl/utils/utils.hpp"

namespace atl {

#define ETROWS "Trajectory [%s] has 0 rows!"
#define ETCOLS "Trajectory [%s] invalid number of cols!"
#define ETLOAD "Failed to load trajectory!"

class Trajectory {
public:
  bool loaded = false;
  int index = -1;

  std::deque<Vec2> pos;
  std::deque<Vec2> vel;
  std::deque<Vec2> inputs;
  std::deque<Vec2> rel_pos;
  std::deque<Vec2> rel_vel;
  Vec3 p0{0.0, 0.0, 0.0};

  Trajectory() {}

  /**
   * Load trajectory
   *
   * @param index Trajectory index
   * @param filepath Trajectory filepath
   * @param pos Robot position in inertial frame
   *
   * @return 0 for success, -1 for failure
   */
  int load(const int index, const std::string &filepath, const Vec3 &pos);

  /**
   * Update trajectory
   *
   * @param pos Robot position in inertial frame
   * @param wp_pos Waypoint position in inertial frame
   * @param wp_vel Waypoint velocity in inertial frame
   * @param wp_input Waypoint inputs for robot
   *
   * @return 0 for success, -1 for failure
   */
  int update(const Vec3 &pos, Vec2 &wp_pos, Vec2 &wp_vel, Vec2 &wp_inputs);

  /**
   * Reset trajectory
   */
  void reset();
};

} // namespace atl
#endif
