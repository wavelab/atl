#ifndef ATL_GIMBAL_HPP
#define ATL_GIMBAL_HPP

#include <iostream>
#include <map>
#include <math.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include "atl/utils/utils.hpp"
#include "atl/vision/gimbal/sbgc.hpp"

namespace atl {

class Gimbal {
public:
  bool configured;

  SBGC sbgc;
  Pose camera_offset;
  double limits[6];
  bool enable_tracking;

  Vec3 setpoints;
  Vec3 target_bpf;

  Vec3 imu_accel;
  Vec3 imu_gyro;
  Vec3 camera_angles;
  Vec3 frame_angles;
  Vec3 rc_angles;
  Vec3 encoder_angles;

  Gimbal();
  ~Gimbal();
  int configure(std::string config_path);
  int on();
  int off();
  static Vec3 getTargetInBF(Pose camera_offset, Vec3 target_cf);
  static Vec3 getTargetInBPF(Pose camera_offset,
                             Vec3 target_cf,
                             Quaternion joint_if);
  static Vec3 getTargetInIF(Vec3 target_bpf,
                            Vec3 gimbal_position,
                            Quaternion gimbal_frame_if);
  int getTargetInBPF(Vec3 target_cf, Vec3 &target_bpf);
  int trackTarget(Vec3 target_cf);
  int updateGimbalStates();
  int setAngle(double roll, double pitch);
  void printSetpoints();
};

} // namespace atl
#endif
