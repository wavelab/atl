#include "atl/control/trajectory_controller.hpp"

namespace atl {

int TrajectoryController::configure(const std::string &config_file) {
  std::string traj_index_file;
  std::string blackbox_file;

  // load config
  ConfigParser parser;
  parser.addParam("vx_controller.k_p", &this->vx_k_p);
  parser.addParam("vx_controller.k_i", &this->vx_k_i);
  parser.addParam("vx_controller.k_d", &this->vx_k_d);
  parser.addParam("vx_controller.min", &this->pitch_limit[0]);
  parser.addParam("vx_controller.max", &this->pitch_limit[1]);

  parser.addParam("vy_controller.k_p", &this->vy_k_p);
  parser.addParam("vy_controller.k_i", &this->vy_k_i);
  parser.addParam("vy_controller.k_d", &this->vy_k_d);
  parser.addParam("vy_controller.min", &this->roll_limit[0]);
  parser.addParam("vy_controller.max", &this->roll_limit[1]);

  parser.addParam("vz_controller.k_p", &this->vz_k_p);
  parser.addParam("vz_controller.k_i", &this->vz_k_i);
  parser.addParam("vz_controller.k_d", &this->vz_k_d);
  parser.addParam("vz_controller.min", &this->throttle_limit[0]);
  parser.addParam("vz_controller.max", &this->throttle_limit[1]);

  parser.addParam("trajectory_index", &traj_index_file);
  parser.addParam("trajectory_threshold", &this->trajectory_threshold);

  parser.addParam("blackbox_enable", &this->blackbox_enable);
  parser.addParam("blackbox_rate", &this->blackbox_rate, true);
  parser.addParam("blackbox_file", &blackbox_file, true);
  if (parser.load(config_file) != 0) {
    return -1;
  }

  // load trajectory index
  std::string config_dir = std::string(dirname((char *) config_file.c_str()));
  paths_combine(config_dir, traj_index_file, traj_index_file);
  if (this->traj_index.load(traj_index_file) != 0) {
    return -2;
  }

  // prepare blackbox file
  if (this->blackbox_enable) {
    if (blackbox_file == "") {
      LOG_ERROR("blackbox file is not set!");
      return -3;
    } else if (this->prepBlackbox(blackbox_file) != 0) {
      LOG_ERROR("Failed to open blackbox file at [%s]", blackbox_file.c_str());
      return -3;
    }

    if (this->blackbox_rate == FLT_MAX) {
      LOG_ERROR("blackbox rate is not set!");
      return -3;
    }
  }

  // convert roll and pitch limits from degrees to radians
  this->roll_limit[0] = deg2rad(this->roll_limit[0]);
  this->roll_limit[1] = deg2rad(this->roll_limit[1]);
  this->pitch_limit[0] = deg2rad(this->pitch_limit[0]);
  this->pitch_limit[1] = deg2rad(this->pitch_limit[1]);

  // initialize setpoints to zero
  this->setpoints << 0.0, 0.0, 0.0;
  this->outputs << 0.0, 0.0, 0.0, 0.0;

  this->configured = true;
  return 0;
}

int TrajectoryController::loadTrajectory(const Vec3 &pos,
                                         const Vec3 &target_pos_B,
                                         const double v) {
  int retval;

  // find trajectory
  retval = this->traj_index.find(pos, v, this->trajectory);
  UNUSED(target_pos_B);

  // check retval
  if (retval == -2) {
    LOG_ERROR(ETIFAIL, pos(2), v);
    return -1;
  } else if (retval == -3) {
    LOG_ERROR(ETLOAD);
    return -1;
  } else {
    LOG_INFO(TLOAD, pos(2), v);
  }

  return 0;
}

int TrajectoryController::prepBlackbox(const std::string &blackbox_file) {
  // setup
  this->blackbox.open(blackbox_file);
  if (!this->blackbox) {
    return -1;
  }

  // write header
  // clang-format off
  this->blackbox << "dt" << ",";
  this->blackbox << "x" << ",";
  this->blackbox << "y" << ",";
  this->blackbox << "z" << ",";
  this->blackbox << "vx" << ",";
  this->blackbox << "vy" << ",";
  this->blackbox << "vz" << ",";
  this->blackbox << "wp_pos_x" << ",";
  this->blackbox << "wp_pos_z" << ",";
  this->blackbox << "wp_vel_x" << ",";
  this->blackbox << "wp_vel_z" << ",";
  this->blackbox << "target_x_B" << ",";
  this->blackbox << "target_y_B" << ",";
  this->blackbox << "target_z_B" << ",";
  this->blackbox << "target_vx_B" << ",";
  this->blackbox << "target_vy_B" << ",";
  this->blackbox << "target_vz_B" << ",";
  this->blackbox << "roll";
  this->blackbox << "pitch";
  this->blackbox << "yaw";
  this->blackbox << std::endl;
  // clang-format on

  return 0;
}

int TrajectoryController::recordTrajectoryIndex() {
  this->blackbox << "trajectory index: " << this->trajectory.index;
  this->blackbox << std::endl;
  return 0;
}

int TrajectoryController::record(const Vec3 &pos,
                                 const Vec3 &vel,
                                 const Vec2 &wp_pos,
                                 const Vec2 &wp_vel,
                                 const Vec2 &wp_inputs,
                                 const Vec3 &target_pos_B,
                                 const Vec3 &target_vel_B,
                                 const Vec3 &rpy,
                                 const double thrust,
                                 const double dt) {
  // pre-check
  this->blackbox_dt += dt;
  if (this->blackbox_enable && this->blackbox_dt > this->blackbox_rate) {
    return 0;
  }

  // record
  this->blackbox << dt << ",";
  this->blackbox << pos(0) << ",";
  this->blackbox << pos(1) << ",";
  this->blackbox << pos(2) << ",";
  this->blackbox << vel(0) << ",";
  this->blackbox << vel(1) << ",";
  this->blackbox << vel(2) << ",";
  this->blackbox << wp_pos(0) << ",";
  this->blackbox << wp_pos(1) << ",";
  this->blackbox << wp_vel(0) << ",";
  this->blackbox << wp_vel(1) << ",";
  this->blackbox << wp_inputs(0) << ",";
  this->blackbox << wp_inputs(1) << ",";
  this->blackbox << target_pos_B(0) << ",";
  this->blackbox << target_pos_B(1) << ",";
  this->blackbox << target_pos_B(2) << ",";
  this->blackbox << target_vel_B(0) << ",";
  this->blackbox << target_vel_B(1) << ",";
  this->blackbox << target_vel_B(2) << ",";
  this->blackbox << rpy(0) << ",";
  this->blackbox << rpy(1) << ",";
  this->blackbox << rpy(2) << ",";
  this->blackbox << thrust;
  this->blackbox << std::endl;
  this->blackbox_dt = 0.0;

  return 0;
}

Vec4 TrajectoryController::calculateVelocityErrors(const Vec3 &v_errors_B,
                                                   const Vec3 &p_errors_B,
                                                   const double yaw_W,
                                                   const double dt) {
  UNUSED(yaw_W);

  // check rate
  this->dt += dt;
  if (this->dt < 0.01) {
    return this->outputs;
  }

  // roll
  double r = this->vy_k_p * v_errors_B(1);
  r += this->vy_k_i * p_errors_B(1);
  r += this->vy_k_d * (v_errors_B(1) - this->vy_error_prev) / this->dt;
  r = -1 * r;

  // pitch
  this->vx_error_sum += v_errors_B(0);
  double p = this->vx_k_p * v_errors_B(0);
  p += this->vx_k_i * this->vx_error_sum;
  p += this->vx_k_d * (v_errors_B(0) - this->vx_error_prev) / this->dt;

  // yaw
  double y = 0.0;

  // throttle
  double t = this->vz_k_p * v_errors_B(2);
  t += this->vz_k_i * p_errors_B(2);
  t += this->vz_k_d * (v_errors_B(2) - this->vz_error_prev) / this->dt;
  t /= fabs(cos(r) * cos(p)); // adjust throttle for roll and pitch

  // keep track of previous errors
  this->vx_error_prev = v_errors_B(0);
  this->vy_error_prev = v_errors_B(1);
  this->vz_error_prev = v_errors_B(2);

  // limit roll, pitch, throttle
  r = (r < this->roll_limit[0]) ? this->roll_limit[0] : r;
  r = (r > this->roll_limit[1]) ? this->roll_limit[1] : r;
  p = (p < this->pitch_limit[0]) ? this->pitch_limit[0] : p;
  p = (p > this->pitch_limit[1]) ? this->pitch_limit[1] : p;
  t = (t < this->throttle_limit[0]) ? this->throttle_limit[0] : t;
  t = (t > this->throttle_limit[1]) ? this->throttle_limit[1] : t;

  // keep track of setpoints and outputs
  this->setpoints = v_errors_B;
  this->outputs << r, p, y, t;
  this->dt = 0.0;

  return this->outputs;
}

int TrajectoryController::update(const Vec3 &target_pos_B,
                                 const Vec3 &target_vel_B,
                                 const Vec3 &pos_W,
                                 const Vec3 &vel_W,
                                 const Quaternion &orientation_W,
                                 const double yaw_W,
                                 const double dt) {
  // obtain position and velocity waypoints
  Vec2 wp_pos, wp_vel, wp_inputs;
  int retval = this->trajectory.update(pos_W, wp_pos, wp_vel, wp_inputs);
  if (retval != 0) {
    LOG_ERROR("Trajectory update failed!");
    return -1;
  }
  Vec2 wp_rel_pos = this->trajectory.rel_pos.at(0);
  // Vec2 wp_rel_vel = this->trajectory.rel_vel.at(0);

  // calculate velocity in body frame
  const Vec3 vel_B = T_B_W{orientation_W} * vel_W;

  // calculate velocity errors (inertial version)
  Vec3 v_errors_B;
  v_errors_B(0) = wp_vel(0) - vel_B(0);
  v_errors_B(1) = target_vel_B(1);
  v_errors_B(2) = wp_vel(1) - vel_B(2);

  // calculate position errors
  Vec3 p_errors_B;
  p_errors_B(0) = wp_rel_pos(0) + target_pos_B(0);
  p_errors_B(1) = target_pos_B(1);
  p_errors_B(2) = wp_rel_pos(1) - pos_W(2);

  // calculate feed-back controls
  this->outputs =
      this->calculateVelocityErrors(v_errors_B, p_errors_B, yaw_W, dt);

  // add in feed-forward controls
  this->outputs(0) += 0.0;          // roll
  this->outputs(1) += wp_inputs(1); // pitch
  this->outputs(2) += yaw_W;        // yaw
  this->outputs(3) += wp_inputs(0); // thrust

  // record
  const Vec3 rpy = quatToEuler321(orientation_W);
  this->record(pos_W,
               vel_W,
               wp_pos,
               wp_vel,
               wp_inputs,
               target_pos_B,
               target_vel_B,
               rpy,
               this->outputs(3),
               dt);

  // check if we are too far off track with trajectory
  if (p_errors_B(0) > this->trajectory_threshold(0)) {
    LOG_ERROR("Trajectory error in the x-axis is too large!");
    LOG_ERROR("error: %f > %f", p_errors_B(0), this->trajectory_threshold(0));
    return -1;
  } else if (p_errors_B(1) > this->trajectory_threshold(1)) {
    LOG_ERROR("Trajectory error in the y-axis is too large!");
    LOG_ERROR("error: %f > %f", p_errors_B(1), this->trajectory_threshold(1));
    return -1;
  } else if (p_errors_B(2) > this->trajectory_threshold(2)) {
    LOG_ERROR("Trajectory error in the z-axis is too large!");
    LOG_ERROR("error: %f > %f", p_errors_B(2), this->trajectory_threshold(2));
    return -1;
  }

  return 0;
}

void TrajectoryController::reset() {
  this->vx_error_prev = 0.0;
  this->vy_error_prev = 0.0;
  this->vz_error_prev = 0.0;

  this->vx_error_sum = 0.0;
}

void TrajectoryController::printOutputs() {
  double r, p, t;

  r = rad2deg(this->outputs(0));
  p = rad2deg(this->outputs(1));
  t = this->outputs(3);

  std::cout << "roll: " << std::setprecision(2) << r << "\t";
  std::cout << "pitch: " << std::setprecision(2) << p << "\t";
  std::cout << "throttle: " << std::setprecision(2) << t << std::endl;
}

} // namespace atl
