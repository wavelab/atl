#ifndef __AWESOMO_UTILS_DATA_HPP__
#define __AWESOMO_UTILS_DATA_HPP__

#include "awesomo_core/utils/math.hpp"


namespace awesomo {

class Pose {
public:
  Quaternion q;
  Vec3 position;

  Pose(void);
  Pose(float roll, float pitch, float yaw, float x, float y, float z);
  Pose(Quaternion q, Vec3 position);

  Eigen::Matrix3d rotationMatrix(void);
};

class Velocity {
public:
  double linear_x;
  double linear_y;
  double linear_z;

  double angular_x;
  double angular_y;
  double angular_z;

  Velocity(void)
      : linear_x(0),
        linear_y(0),
        linear_z(0),
        angular_x(0),
        angular_y(0),
        angular_z(0) {}
};

class Attitude {
public:
  double x;
  double y;
  double z;
  double w;

  double roll;
  double pitch;
  double yaw;

  Attitude(void) : x(0), y(0), z(0), w(0), roll(0), pitch(0), yaw(0) {}
};


class LandingTargetPosition {
public:
  bool detected;
  Vec3 position;

  LandingTargetPosition(void) : detected(false), position(0, 0, 0) {}
};

}  // end of awesomo namespace
#endif
