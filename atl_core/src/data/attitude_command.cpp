#include "atl/data/attitude_command.hpp"

namespace atl {

AttitudeCommand::AttitudeCommand(Vec4 command) {
  // quaternion
  Vec3 euler{command(0), command(1), command(2)}; // roll, pitch, yaw
  euler2quat(euler, 321, this->orientation);

  // throttle
  this->throttle = command(3);
}

Vec3 AttitudeCommand::toEuler(const std::string &coordinate_system) {
  if (coordinate_system == "NED") {
    // transform orientation from NWU to NED
    Quaternion q_ned = nwu2ned(this->orientation);

    // convert to euler
    Vec3 euler_ned;
    quat2euler(q_ned, 321, euler_ned);
    return euler_ned;

  } else {
    // convert to euler
    Vec3 euler_nwu;
    quat2euler(this->orientation, 321, euler_nwu);
    return euler_nwu;
  }
}

void AttitudeCommand::print() {
  Vec3 euler;
  quat2euler(this->orientation, 321, euler);

  printf("roll: %.2f\t", euler(0));
  printf("pitch: %.2f\t", euler(1));
  printf("yaw: %.2f\t", euler(2));
  printf("throttle: %.2f\n", this->throttle);
}

} // namespace atl
