#ifndef __QUADROTOR_HPP__
#define __QUADROTOR_HPP__

#include <iostream>

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/controller.hpp"
#include "awesomo_core/estimation/estimator.hpp"

namespace awesomo {

// #define YAW_CONTROL_ON

// CONSTANTS
#define LZ_THRESHOLD 0

// STATES
#define TARGET_LOST -2
#define MISSION_ACCOMPLISHED -1
#define IDLE_MODE 0
#define DISCOVER_MODE 1
#define TRACKING_MODE 2
#define LANDING_MODE 3

// TIMEOUTS
#define TARGET_LOST_TIMEOUT 2  // run on kalman estimate until this time
#define LANDING_DELAY 5        // observe the target before landing


class LandingConfig {
public:
  // height update
  float period;
  float descend_multiplier;
  float recover_multiplier;

  // disarm conditions
  float belief_threshold;
  Eigen::Vector3d cutoff_position;

  // constructor
  LandingConfig(void);
  LandingConfig(float period,
                float desend_multiplier,
                float recover_multiplier,
                float belief_threshold,
                Eigen::Vector3d cutoff_position);
};

class Quadrotor {
public:
  // configs
  LandingConfig *landing_config;

  // state
  int mission_state;
  Pose world_pose;
  double yaw;

  bool height_offset_initialized;
  float height_offset;

  float hover_height_original;
  float hover_height;

  int landing_belief;
  struct timespec tracking_start;
  struct timespec target_last_updated;
  struct timespec height_last_updated;
  std::vector<Eigen::Vector2d> lt_history;

  // controllers
  CarrotController *carrot_controller;
  PositionController *position_controller;

  // estimators
  bool estimator_initialized;
  struct kf tag_estimator;

  Quadrotor(std::map<std::string, std::string> configs);
  int loadConfig(std::string config_file_path);
  Attitude positionControllerCalculate(Eigen::Vector3d setpoint,
                                       Pose robot_pose,
                                       float yaw,
                                       float dt);

  void resetPositionController(void);
  int calculateLandingTargetYaw(double *yaw);
  void runDiscoverMode(LandingTargetPosition landing);
  int checkLandingTargetEstimation(Eigen::Vector3d &est);
  void runTrackingModeBPF(LandingTargetPosition landing, float dt);
  bool withinLandingZone(Eigen::Vector3d &m, Eigen::Vector3d &e);
  bool withinLandingZone(Eigen::Vector3d &m);
  void runLandingMode(LandingTargetPosition landing, float dt);
  int runMission(Pose robot_pose, LandingTargetPosition landing, float dt);
};

}  // end of awesomo namespace
#endif
