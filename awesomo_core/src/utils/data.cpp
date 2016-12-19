#include "awesomo_core/utils/data.hpp"


namespace awesomo {

Pose::Pose(void) {
  Quaternion q;

  this->q = q.setIdentity();
  this->position = Vec3::Zero(3, 1);
}

Pose::Pose(float roll, float pitch, float yaw, float x, float y, float z) {
  euler2Quaternion(roll, pitch, yaw, this->q);
  this->position << x, y, z;
}

Pose::Pose(Quaternion q, Vec3 position) {
  this->q = q;
  this->position = position;
}

Eigen::Matrix3d Pose::rotationMatrix(void) {
  return this->q.toRotationMatrix();
}

}  // end of awesomo namespace
