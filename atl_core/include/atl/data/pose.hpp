#ifndef ATL_DATA_POSE_HPP
#define ATL_DATA_POSE_HPP

#include "atl/data/transform.hpp"
#include "atl/utils/utils.hpp"

namespace atl {

/**
 * Pose stores orientation and position information
 */
struct Pose {
  Vec3 position{0.0, 0.0, 0.0};
  Quaternion orientation{1.0, 0.0, 0.0, 0.0};

  Pose() {}
  Pose(Vec3 position, Quaternion orientation)
      : position{position}, orientation{orientation} {}
  Pose(double roll, double pitch, double yaw, double x, double y, double z);

  /// Obtain orientation as a rotation matrix
  Mat3 rotationMatrix();

  /// Print position
  void printPosition();

  /// Print orientation
  void printOrientation();

  /// Print quaternion
  void printQuaternion();

  /// Print pose
  void print();
};

} // namespace atl
#endif
