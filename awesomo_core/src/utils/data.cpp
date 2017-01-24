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
  this->position = Vec3::Zero(3, 1);
  this->orientation = Quaternion();
}

Pose::Pose(Vec3 position, Quaternion orientation) {
  this->position = position;
  this->orientation = orientation;
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
  euler2quat(euler, 321, this->orientation);
  this->position << x, y, z;
}
// clang-format on

Mat3 Pose::rotationMatrix(void) {
  return this->orientation.toRotationMatrix();
}

void Pose::printPosition(void) {
  std::cout << "position: [";
  std::cout << "%.2f, ", this->position(0);
  std::cout << "%.2f, ", this->position(1);
  std::cout << "%.2f", this->position(2);
  std::cout << "]" << std::endl;
}

void Pose::printOrientation(void) {
  Vec3 euler;

  quat2euler(this->orientation, 321, euler);
  std::cout << "orientation : [";
  std::cout << "%.2f, ", euler(0);
  std::cout << "%.2f, ", euler(1);
  std::cout << "%.2f", euler(2);
  std::cout << "]" << std::endl;
}

void Pose::print(void) {
  this->printPosition();
  this->printOrientation();
}

}  // end of awesomo namespace
