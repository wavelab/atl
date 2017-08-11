#include "atl/control/trajectory_index.hpp"

namespace atl {

int TrajectoryIndex::load(const std::string &index_file,
                          const double pos_thres,
                          const double vel_thres) {
  // pre-check
  if (file_exists(index_file) == false) {
    LOG_ERROR("File not found: %s", index_file.c_str());
    return -1;
  }

  // load trajectory index
  // assumes each column is: (index, p0_x, p0_z, pf_x, pf_z, z)
  csv2mat(index_file, true, this->index_data);
  this->traj_dir = std::string(dirname((char *) index_file.c_str()));
  this->pos_thres = pos_thres;
  this->vel_thres = vel_thres;

  if (this->index_data.rows() == 0) {
    LOG_ERROR(ETIROWS, index_file.c_str());
    return -2;
  } else if (this->index_data.cols() != 3) {
    LOG_ERROR(ETICOLS, index_file.c_str());
    return -2;
  }

  this->loaded = true;
  return 0;
}

int TrajectoryIndex::find(const Vec3 &pos, const double v, Trajectory &traj) {
  bool p_ok, v_ok;
  std::vector<int> matches;
  std::string traj_file;

  // pre-check
  if (this->loaded == false) {
    return -1;
  }

  // NOTE: the following is not the most efficient way of implementing a
  // lookup table, a better way could involve a search tree and traverse it
  // or even a bucket based approach. The following implements a list
  // traversal type search which is approx O(n), ok for small lookups.

  // find rows in the index that have same approx
  // start height (z) and velocity (v)
  for (int i = 0; i < this->index_data.rows(); i++) {
    p_ok = fabs(pos(2) - this->index_data(i, 1)) < this->pos_thres;
    v_ok = fabs(v - this->index_data(i, 2)) < this->vel_thres;

    if (p_ok && v_ok) {
      matches.push_back(i);
    }
  }

  // check number of matches
  if (matches.size() == 0) {
    return -2; // found no trajectory
  }

  // load trajectory
  traj_file = this->traj_dir + "/";
  traj_file += std::to_string((int) matches[0]) + ".csv";
  if (traj.load(matches[0], traj_file, pos) != 0) {
    return -3;
  }

  return 0;
}

} // namespace atl
