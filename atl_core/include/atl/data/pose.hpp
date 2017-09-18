#ifndef ATL_DATA_POSE_HPP
#define ATL_DATA_POSE_HPP

#include "atl/utils/utils.hpp"

namespace atl {

/**
 * Pose stores orientation and position information
 */
struct Pose {
  std::string frame;
  Vec3 position{0.0, 0.0, 0.0};
  Quaternion orientation{1.0, 0.0, 0.0, 0.0};

  Pose() {}

  Pose(const std::string &frame,
       const Vec3 &position,
       const Quaternion &orientation)
      : frame{frame}, position{position}, orientation{orientation} {}

  Pose(const std::string &frame, const Vec3 &position, const Mat3 &orientation)
      : frame{frame}, position{position}, orientation{orientation} {}

  Pose(const std::string &frame,
       const double roll,
       const double pitch,
       const double yaw,
       const double x,
       const double y,
       const double z)
      : frame{frame}, position{x, y, z} {
    Vec3 euler{roll, pitch, yaw};
    this->orientation = euler321ToQuat(euler);
  }

  /// Obtain orientation as a rotation matrix
  Mat3 rotationMatrix() const { return this->orientation.toRotationMatrix(); }

  /// Obtain pose as a transformation matrix
  Mat4 transformMatrix() const {
    const Mat3 R = this->rotationMatrix();
    const Vec3 t = this->position;

    Mat4 T = Mat4::Identity();
    T.block(0, 0, 3, 3) = R;
    T.block(0, 3, 3, 1) = t;

    return T;
  }

  /// Print pose
  friend std::ostream &operator<<(std::ostream &os, const Pose &pose) {
    os << "position [";
    os << "x: " << std::setprecision(2) << pose.position(0) << ", ";
    os << "y: " << std::setprecision(2) << pose.position(1) << ", ";
    os << "z: " << std::setprecision(2) << pose.position(2);
    os << "]" << std::endl;

    os << "quaternion[";
    os << "w: " << std::setprecision(2) << pose.orientation.w() << ", ";
    os << "x: " << std::setprecision(2) << pose.orientation.x() << ", ";
    os << "y: " << std::setprecision(2) << pose.orientation.y() << ", ";
    os << "z: " << std::setprecision(2) << pose.orientation.z();
    os << "]" << std::endl;

    return os;
  }
};

} // namespace atl
#endif
