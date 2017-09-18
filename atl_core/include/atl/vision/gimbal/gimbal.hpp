#ifndef ATL_GIMBAL_HPP
#define ATL_GIMBAL_HPP

#include <iostream>
#include <map>
#include <math.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include "atl/data/data.hpp"
#include "atl/utils/utils.hpp"
#include "atl/vision/gimbal/sbgc.hpp"

namespace atl {

class Gimbal {
public:
  bool configured = false;

  SBGC sbgc;
  Pose camera_offset;
  double limits[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool enable_tracking = false;

  Vec3 setpoints = Vec3{0.0, 0.0, 0.0};
  Vec3 target_P = Vec3{0.0, 0.0, 0.0};

  Vec3 imu_accel = Vec3{0.0, 0.0, 0.0};
  Vec3 imu_gyro = Vec3{0.0, 0.0, 0.0};
  Vec3 camera_angles = Vec3{0.0, 0.0, 0.0};
  Vec3 frame_angles = Vec3{0.0, 0.0, 0.0};
  Vec3 rc_angles = Vec3{0.0, 0.0, 0.0};
  Vec3 encoder_angles = Vec3{0.0, 0.0, 0.0};

  Gimbal() {}
  ~Gimbal() { this->off(); }

  int configure(const std::string &config_path);
  int on();
  int off();
  static Vec3 getTargetInBF(const Pose &camera_offset, const Vec3 &target_C);
  static Vec3 getTargetInBPF(const Pose &camera_offset,
                             const Vec3 &target_C,
                             const Quaternion &joint_W);
  static Vec3 getTargetInIF(const Vec3 &target_P,
                            const Vec3 &gimbal_position,
                            const Quaternion &gimbal_frame_W);
  int getTargetInBPF(const Vec3 &target_C, Vec3 &target_P);
  int trackTarget(const Vec3 &target_P);
  int updateGimbalStates();
  int setAngle(const double roll, const double pitch);
  void printSetpoints();
};

} // namespace atl
#endif
