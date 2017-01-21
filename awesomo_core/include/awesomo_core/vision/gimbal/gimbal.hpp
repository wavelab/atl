#ifndef __AWESOMO_GIMBAL_HPP__
#define __AWESOMO_GIMBAL_HPP__

#include <iostream>
#include <math.h>
#include <map>
#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/vision/gimbal/sbgc.hpp"

#define G 9.80665

namespace awesomo {

class Gimbal {
public:
  bool configured;

  SBGC sbgc;
  Pose camera_offset;
  double limits[6];
  Vec3 setpoints;
  Vec3 target_bpf;

  // Gimbal data
  Vec3 imu_accel;
  Vec3 imu_gyro;
  Vec3 camera_angles;
  Vec3 frame_angles;
  Vec3 rc_angles;

  Gimbal(void);
  ~Gimbal(void);
  int configure(std::string config_path);
  int updateGimbalStates(void);
  static Vec3 getTargetInBF(Pose camera, Vec3 target_cf);
  static Vec3 getTargetInBPF(Pose camera, Vec3 target_cf, Quaternion &imu_if);
  int getTargetInBPF(Vec3 target_cf, Vec3 &target_bpf);
  int trackTarget(Vec3 target_cf);
  int shutdownMotors(void);
  void printSetpoints(void);
  int setAngle(double roll, double pitch);
};

}  // end of awesomo namespace
#endif
