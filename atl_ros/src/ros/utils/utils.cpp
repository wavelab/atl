#include "atl/ros/utils/utils.hpp"

namespace atl {
namespace ros {

int ros2gaz(Vec3 ros, Vec3 &gaz) {
  Mat3 R;

  // clang-format off
  R << 0.0, 1.0, 0.0,
       -1.0, 0.0, 0.0,
       0.0, 0.0, 1.0;
  // clang-format on

  // transform gazebo position to ros
  // ENU to NWU
  gaz = R * ros;

  return 0;
}

int ros2gaz(Quaternion ros_quat, Quaternion &gaz_quat) {
  gaz_quat.w() = ros_quat.w();
  gaz_quat.x() = -ros_quat.y();
  gaz_quat.y() = ros_quat.x();
  gaz_quat.z() = ros_quat.z();
  gaz_quat.normalize();

  return 0;
}

int gaz2ros(Vec3 gaz, Vec3 &ros) {
  Mat3 R;

  // clang-format off
  R << 0.0, -1.0, 0.0,
       1.0, 0.0, 0.0,
       0.0, 0.0, 1.0;
  // clang-format on

  // transform gazebo position to ros
  // NWU to ENU
  ros = R * gaz;

  return 0;
}

int gaz2ros(Quaternion gaz_quat, Quaternion &ros_quat) {
  ros_quat.w() = gaz_quat.w();
  ros_quat.x() = -gaz_quat.y();
  ros_quat.y() = gaz_quat.x();
  ros_quat.z() = gaz_quat.z();
  ros_quat.normalize();

  return 0;
}

} // namespace ros
} // namespace atl
