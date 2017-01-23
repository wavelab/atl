#include "awesomo_core/utils/data.hpp"


namespace awesomo {

// ATTITUDE COMMAND
AttitudeCommand::AttitudeCommand(void) {
  this->orientation.w() = 0.0;
  this->orientation.x() = 0.0;
  this->orientation.y() = 0.0;
  this->orientation.z() = 0.0;
  this->throttle = 0.0;
}

AttitudeCommand::AttitudeCommand(Vec4 command) {
  Vec3 euler;
  double throttle;

  // quaternion
  euler << command(0), command(1), command(2);  // roll, pitch, yaw
  euler2quat(euler, 321, this->orientation);

  // throttle
  this->throttle = command(3);
}

void AttitudeCommand::print(void) {
  Vec3 euler;
  quat2euler(this->orientation, 321, euler);

  printf("roll: %.2f\t", euler(0));
  printf("pitch: %.2f\t", euler(1));
  printf("yaw: %.2f\t", euler(2));
  printf("throttle: %.2f\n", this->throttle);
}


// POSE
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
  Vec3 euler;
  euler << roll, pitch, yaw;
  euler2quat(euler, 321, this->q);
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
