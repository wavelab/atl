#ifndef __AWESOMO_CONTROL_LANDING_CONTROLLER_HPP__
#define __AWESOMO_CONTROL_LANDING_CONTROLLER_HPP__

#include <iomanip>
#include <libgen.h>
#include <string>

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/pid_controller.hpp"


namespace awesomo {

#define ETROWS "Trajectory [%s] has 0 rows!"
#define ETCOLS "Trajectory [%s] invalid number of cols!"
#define ETLOAD "Failed to load trajectory!"
#define ETIROWS "Trajectory index [%s] has 0 rows!"
#define ETICOLS "Trajectory index [%s] invalid number of cols!"
#define ETIFAIL "Found no trajectory for p0 = (%f, %f), pf = (%f, %f), v = %f"

class Trajectory {
public:
  bool loaded;
  VecX x, z, vx, vz;
  VecX az,theta;

  Trajectory(void);
  int load(std::string filepath);
  void reset(void);
};

class TrajectoryIndex {
public:
  bool loaded;

  std::string traj_dir;
  MatX index_data;
  double pos_thres;
  double vel_thres;

  TrajectoryIndex(void);
  int load(std::string index_file,
           double pos_thres=0.1,
           double vel_thres=0.2);
  int find(Vec2 p0, Vec2 pf, double v, Trajectory &traj);
};

class LandingController {
public:
  bool configured;

  double pctrl_dt;
  double vctrl_dt;

  PID x_controller;
  PID y_controller;
  PID z_controller;
  double hover_throttle;

  PID vx_controller;
  PID vy_controller;
  PID vz_controller;

  double roll_limit[2];
  double pitch_limit[2];
  double throttle_limit[2];

  Vec3 pctrl_setpoints;
  Vec4 pctrl_outputs;
  Vec3 vctrl_setpoints;
  Vec4 vctrl_outputs;
  AttitudeCommand att_cmd;

  TrajectoryIndex traj_index;
  Trajectory trajectory;

  LandingController(void);
  int configure(std::string config_file);
  int loadTrajectory(Vec3 pos, Vec3 target_pos_bf, double v);
  Vec4 calculatePositionErrors(Vec3 errors, double yaw, double dt);
  Vec4 calculateVelocityErrors(Vec3 errors, double yaw, double dt);
  AttitudeCommand calculate(Vec3 pos_errors,
                            Vec3 vel_errors,
                            double yaw,
                            double dt);
  AttitudeCommand calculate(Vec3 target_pos_bf,
                            Vec3 target_vel_bf,
                            Vec3 pos,
                            Vec3 pos_prev,
                            double yaw,
                            double dt);
  void reset(void);
  void printOutputs(void);
  void printErrors(void);
};

}  // end of awesomo namespace
#endif
