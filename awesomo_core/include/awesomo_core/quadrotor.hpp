#ifndef __QUADROTOR_HPP__
#define __QUADROTOR_HPP__

#include <iostream>

#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/control/control.hpp"
#include "awesomo_core/estimation/estimation.hpp"

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
  std::vector<Vec2> lt_history;

  // controllers
  PositionController position_controller;

  // estimators
  bool estimator_initialized;
  KalmanFilter tag_estimator;

  Quadrotor(void);
  Quadrotor(std::map<std::string, std::string> configs);
  int loadConfig(std::string config_file_path);
  Attitude positionControllerCalculate(Vec3 setpoint,
                                       Pose robot_pose,
                                       float yaw,
                                       float dt);
  void resetPositionController(void);
  int calculateLandingTargetYaw(double *yaw);
  void runDiscoverMode(LandingTargetPosition landing);
  int checkLandingTargetEstimation(Vec3 &est);
  void runTrackingModeBPF(LandingTargetPosition landing, float dt);
  bool withinLandingZone(Vec3 &m, Vec3 &e);
  bool withinLandingZone(Vec3 &m);
  void runLandingMode(LandingTargetPosition landing, float dt);
  int runMission(Pose robot_pose, LandingTargetPosition landing, float dt);
};

}  // end of awesomo namespace
#endif
