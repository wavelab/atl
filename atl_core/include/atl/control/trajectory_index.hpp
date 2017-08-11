#ifndef ATL_CONTROL_TRAJECTORY_INDEX_HPP
#define ATL_CONTROL_TRAJECTORY_INDEX_HPP

#include <libgen.h>
#include <deque>
#include <iomanip>
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
  bool loaded;

  std::string traj_dir;
  MatX index_data;
  double pos_thres;
  double vel_thres;

  TrajectoryIndex()
      : loaded{false},
        traj_dir{""},
        index_data{MatX::Zero(1, 1)},
        pos_thres{0.0},
        vel_thres{0.0} {}

  int load(
    const std::string &index_file,
    double pos_thres = 0.2,
    double vel_thres = 0.2);

  int find(const Vec3 &pos, double v, Trajectory &traj);
};

}  // namespace atl
#endif
