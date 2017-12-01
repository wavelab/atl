#ifndef ATL_CONTROL_TRAJECTORY_INDEX_HPP
#define ATL_CONTROL_TRAJECTORY_INDEX_HPP

#include <deque>
#include <iomanip>
#include <libgen.h>
#include <string>

#include <yaml-cpp/yaml.h>

#include "atl/control/pid.hpp"
#include "atl/control/trajectory.hpp"
#include "atl/utils/utils.hpp"

namespace atl {

#define ETIROWS "Trajectory index [%s] has 0 rows!"
#define ETICOLS "Trajectory index [%s] invalid number of cols!"
#define ETIFAIL "Found no trajectory for z = %f, v = %f"
#define TLOAD "Loaded trajectory @ z = %f, v = %f"

class TrajectoryIndex {
public:
  bool loaded = false;

  std::string traj_dir;
  MatX index_data{MatX::Zero(1, 1)};
  double pos_thres = 0.0;
  double vel_thres = 0.0;

  TrajectoryIndex() {}

  /**
   * Load trajectory index
   *
   * @param index_file Trajectory index file
   * @param pos_thres Position threshold (m)
   * @param vel_thres Velocity threshold (m/s)
   *
   * @return
   *    - 0: Success
   *    - -1: Failed to find index file
   *    - -2: Failed to load index file
   */
  int load(const std::string &index_file,
           const double pos_thres = 0.2,
           const double vel_thres = 0.2);

  /**
   * Find trajectory based on relative position and velocity
   * to landing target
   *
   * @param pos Relative position
   * @param vel Relative velocity
   * @param traj Found trajectory
   *
   * @return
   *    - 0: for success
   *    - -1: Trajectory index not loaded yet
   *    - -2: Found no trajectory
   *    - -3: Failed to load trajectory
   */
  int find(const Vec3 &pos, const double vel, Trajectory &traj);
};

} // namespace atl
#endif
