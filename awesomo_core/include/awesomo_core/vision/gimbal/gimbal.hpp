#ifndef __AWESOMO_GIMBAL_HPP__
#define __AWESOMO_GIMBAL_HPP__

#include <iostream>
#include <math.h>
#include <map>
#include <yaml-cpp/yaml.h>

#include "awesomo_core/utils/utils.hpp"
#include "awesomo_core/vision/gimbal/sbgc.hpp"


namespace awesomo {

class Gimbal {
public:
  bool configured;
  bool sim_mode;

  SBGC sbgc;
  Pose camera_offset;
  double limits[6];
  Vec3 setpoints;
  Vec3 target_bpf;

  Gimbal(void);
  int configure(std::string config_path);
  Vec3 getTargetPositionInBodyFrame(Vec3 target_cf);
  Vec3 getTargetPositionInBodyPlanarFrame(Vec3 target_cf, Quaternion &imu_if);
  int getTargetPositionInBodyPlanarFrame(Vec3 target_cf, Vec3 &target_bpf);
  int trackTarget(Vec3 target_cf);
  int setAngle(double roll, double pitch);
  void printSetpoints(void);
};

}  // end of awesomo namespace
#endif
