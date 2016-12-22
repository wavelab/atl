#include "awesomo_core/utils/data.hpp"


namespace awesomo {

Pose::Pose(void) {
  Quaternion q;

  this->q = q.setIdentity();
  this->position = Vec3::Zero(3, 1);
}

Pose::Pose(Quaternion q, Vec3 position) {
  this->q = q;
  this->position = position;
}

// clang-format off
Pose::Pose(double roll,
           double pitch,
           double yaw,
           double x,
           double y,
           double z) {
  euler2Quaternion(roll, pitch, yaw, this->q);
  this->position << x, y, z;
}
// clang-format on

void Pose::printPosition(void) {
  printf("position: [");
  printf("%.2f, ", this->position(0));
  printf("%.2f, ", this->position(1));
  printf("%.2f", this->position(2));
  printf("]\n");
}

void Pose::printOrientation(void) {
  printf("quaternion: [");
  printf("%.2f, ", this->q.w());
  printf("%.2f, ", this->q.x());
  printf("%.2f,", this->q.y());
  printf("%.2f", this->q.z());
  printf("]\n");
}

void Pose::print(void) {
  this->printPosition();
  this->printOrientation();
}

Eigen::Matrix3d Pose::rotationMatrix(void) {
  return this->q.toRotationMatrix();
}

}  // end of awesomo namespace
