#include "atl/control/trajectory.hpp"

namespace atl {

int Trajectory::load(const int index,
                     const std::string &filepath,
                     const Vec3 &p0) {
  MatX traj_data;
  Vec2 p, v, u, rel_p, rel_v;

  // pre-check
  if (file_exists(filepath) == false) {
    LOG_ERROR("File not found: %s", filepath.c_str());
    return -1;
  }

  // load trajectory file
  // assumes each column is:
  // - x
  // - vx
  // - z
  // - vz
  // - az
  // - theta
  // - rel_x
  // - rel_z
  // - rel_vx
  // - rel_vz
  this->reset();
  this->index = index;
  csv2mat(filepath, true, traj_data);
  if (traj_data.rows() == 0) {
    LOG_ERROR(ETROWS, filepath.c_str());
    return -2;
  } else if (traj_data.cols() != 10) {
    LOG_ERROR(ETCOLS, filepath.c_str());
    return -2;
  }

  // set trajectory class
  for (int i = 0; i < traj_data.rows(); i++) {
    p << traj_data(i, 0), traj_data(i, 2);     // x, z
    v << traj_data(i, 1), traj_data(i, 3);     // vx, vz
    u << traj_data(i, 4), traj_data(i, 5);     // az, theta
    rel_p << traj_data(i, 6), traj_data(i, 7); // rel_x, rel_z
    rel_v << traj_data(i, 8), traj_data(i, 9); // rel_vx, rel_vz

    this->pos.push_back(p);
    this->vel.push_back(v);
    this->inputs.push_back(u);
    this->rel_pos.push_back(rel_p);
    this->rel_vel.push_back(rel_v);
  }

  this->p0 = p0;
  this->loaded = true;
  return 0;
}

int Trajectory::update(const Vec3 &pos,
                       Vec2 &wp_pos,
                       Vec2 &wp_vel,
                       Vec2 &wp_inputs) {
  // pre-check
  if (this->loaded == false) {
    return -1;
  } else if (this->pos.size() < 2) {
    wp_pos = this->pos.at(0);
    wp_vel = this->vel.at(0);
    wp_inputs = this->inputs.at(0);
    return 0;
  }

  // setup
  Vec2 wp_pos_start = this->pos.at(0);
  Vec2 wp_pos_end = this->pos.at(1);

  Vec2 wp_vel_start = this->vel.at(0);
  Vec2 wp_vel_end = this->vel.at(1);

  Vec2 wp_inputs_start = this->inputs.at(0);
  Vec2 wp_inputs_end = this->inputs.at(1);

  Vec2 q_pos;
  q_pos(0) = (this->p0.block(0, 0, 2, 1) - pos.block(0, 0, 2, 1)).norm();
  q_pos(1) = pos(2);

  // find next waypoint position, velocity and inputs
  double wp_percent = closest_point(wp_pos_start, wp_pos_end, q_pos, wp_pos);
  wp_vel = linear_interpolation(wp_vel_start, wp_vel_end, wp_percent);
  wp_inputs = linear_interpolation(wp_inputs_start, wp_inputs_end, wp_percent);

  // update trajectory waypoints
  if (wp_percent > 1.0) {
    this->pos.pop_front();
    this->vel.pop_front();
    this->inputs.pop_front();
    this->rel_pos.pop_front();
    this->rel_vel.pop_front();
  }

  // debug
  // std::cout << "pos: " << pos.transpose() << std::endl;
  // std::cout << "wp_start: " << wp_pos_start.transpose() << std::endl;
  // std::cout << "wp_end: " << wp_pos_end.transpose() << std::endl;
  // std::cout << "wp_vel: " << wp_vel.transpose() << std::endl;
  // std::cout << "wp: " << wp_pos.transpose() << std::endl;
  // std::cout << "rel_pos: " << this->rel_pos.at(0).transpose() << std::endl;
  // std::cout << "rel_vel: " << this->rel_vel.at(0).transpose() << std::endl;
  // std::cout << "inputs: " << this->inputs.at(0).transpose() << std::endl;
  // std::cout << "nb waypoints left: " << this->pos.size() << std::endl;
  // std::cout << std::endl;

  return 0;
}

void Trajectory::reset() {
  this->loaded = false;
  this->pos.clear();
  this->vel.clear();
  this->inputs.clear();
  this->rel_pos.clear();
  this->rel_vel.clear();
  this->p0 << 0.0, 0.0, 0.0;
}

} // namespace atl
