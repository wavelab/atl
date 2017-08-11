#ifndef ATL_CONTROL_TRAJECTORY_HPP
#define ATL_CONTROL_TRAJECTORY_HPP

#include <libgen.h>
#include <deque>
#include <iomanip>
#include <string>

#include <yaml-cpp/yaml.h>

#include "atl/utils/utils.hpp"

namespace atl {

#define ETROWS "Trajectory [%s] has 0 rows!"
#define ETCOLS "Trajectory [%s] invalid number of cols!"
#define ETLOAD "Failed to load trajectory!"

class Trajectory {
public:
  bool loaded;
  int index;

  std::deque<Vec2> pos;
  std::deque<Vec2> vel;
  std::deque<Vec2> inputs;
  std::deque<Vec2> rel_pos;
  std::deque<Vec2> rel_vel;
  Vec3 p0;

  Trajectory() : loaded{false}, index{-1}, p0{Vec3::Zero()} {}
  int load(int index, const std::string &filepath, const Vec3 &pos);
  int update(Vec3 pos, Vec2 &wp_pos, Vec2 &wp_vel, Vec2 &wp_inputs);
  void reset();
};

}  // namespace atl
#endif
