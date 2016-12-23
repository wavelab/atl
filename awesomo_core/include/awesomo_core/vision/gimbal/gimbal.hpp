#ifndef __AWESOMO_GIMBAL_HPP__
#define __AWESOMO_GIMBAL_HPP__

#include <iostream>
#include <math.h>
#include <map>
#include <yaml-cpp/yaml.h>

#include "awesomo_core/vision/util.hpp"
#include "awesomo_core/vision/sbgc.hpp"


class GimbalLimit {
public:
  Eigen::Vector2d roll_limits;
  Eigen::Vector2d pitch_limits;
  Eigen::Vector2d yaw_limits;
};

class Gimbal {
public:
  SBGC *sbgc;
  Pose pose;
  GimbalLimit gimbal_limits;

  Gimbal(void){};
  Gimbal(float roll, float pitch, float yaw, float x, float y, float z);
  Gimbal(std::string config_path);

  Eigen::Vector3d getTargetPositionBFrame(Eigen::Vector3d target_position);
  Eigen::Vector3d getTargetPositionBPFrame(Eigen::Vector3d target_position,
                                           Eigen::Quaterniond &imu);
  int transformTargetPosition(Eigen::Vector3d target,
                              Eigen::Vector3d &transformed_target);
  int setGimbalLimits(float roll_upper,
                      float roll_lower,
                      float pitch_upper,
                      float pitch_lower,
                      float yaw_upper,
                      float yaw_lower);
  int checkLimits(float &value, Eigen::Vector2d limits);
  int checkSetPointLimits(Eigen::Vector3d frame_rpy,
                          float roll,
                          float pitch,
                          float yaw);
  int trackTarget(Eigen::Vector3d target);
  int setGimbalAngles(double roll, double pitch, double yaw);
};


#endif
