#ifndef ATL_UTILS_DATA_HPP
#define ATL_UTILS_DATA_HPP

#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "atl/utils/math.hpp"

namespace atl {

// CSV ERROR MESSAGES
#define E_CSV_DATA_LOAD "Error! failed to load test data [%s]!!\n"
#define E_CSV_DATA_OPEN "Error! failed to open file for output [%s]!!\n"

/**
 * Attitude Command
 */
class AttitudeCommand {
public:
  Quaternion orientation;
  double throttle;

  AttitudeCommand() : orientation{1.0, 0.0, 0.0, 0.0}, throttle{0.0} {}
  AttitudeCommand(Vec4 command);
  void print();
};

/**
 * Pose stores orientation and position information
 */
class Pose {
public:
  Vec3 position;
  Quaternion orientation;

  Pose() : position{0.0, 0.0, 0.0}, orientation{1.0, 0.0, 0.0, 0.0} {}
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

/**
 * Twist stores linear and angular velocity
 */
class Twist {
public:
  Vec3 linear;
  Vec3 angular;

  Twist() : linear{VecX::Zero(3)}, angular{VecX::Zero(3)} {}
};

class LandingTargetPosition {
public:
  bool detected;
  Vec3 position;

  LandingTargetPosition() {
    this->detected = false;
    this->position << 0, 0, 0;
  }
};

int csvrows(std::string file_path);
int csvcols(std::string file_path);
int csv2mat(std::string file_path, bool header, MatX &data);
int mat2csv(std::string file_path, MatX data);

}  // namespace atl
#endif
