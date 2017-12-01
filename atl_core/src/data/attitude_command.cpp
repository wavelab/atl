#include "atl/data/attitude_command.hpp"

namespace atl {

AttitudeCommand::AttitudeCommand(Vec4 command) {
  // roll, pitch, yaw
  this->rpy << command(0), command(1), command(2);

  // throttle
  this->throttle = command(3);
}

Vec3 AttitudeCommand::toEuler(const std::string &coordinate_system) {
  if (coordinate_system == "NED") {
    // Transform NWU to NED
    return nwu2ned(this->rpy);

  } else if (coordinate_system == "ENU") {
    // Transform NWU to ENU
    this->rpy(2) += M_PI / 2.0;
    return this->rpy;

  } else {
    // NWU euler angles
    return this->rpy;
  }
}

void AttitudeCommand::print() {
  printf("roll: %.2f\t", this->rpy(0));
  printf("pitch: %.2f\t", this->rpy(1));
  printf("yaw: %.2f\t", this->rpy(2));
  printf("throttle: %.2f\n", this->throttle);
}

} // namespace atl
