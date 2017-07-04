#ifndef ATL_VISION_APRILTAG_DATA_HPP
#define ATL_VISION_APRILTAG_DATA_HPP

#include "atl/utils/utils.hpp"

namespace atl {

class TagPose {
public:
  int id;
  bool detected;
  Vec3 position;
  Quaternion orientation;

  TagPose(void) {
    this->id = -1;
    this->detected = false;
    this->position << 0.0, 0.0, 0.0;
    this->orientation = Quaternion();
  };

  TagPose(int id, bool detected, Vec3 position, Quaternion orientation) {
    this->id = id;
    this->detected = detected;
    this->position = position;
    this->orientation = orientation;
  }

  TagPose(int id, bool detected, Vec3 position, Mat3 rotmat) {
    this->id = id;
    this->detected = detected;
    this->position = position;
    this->orientation = Quaternion(rotmat);
  }

  void print(void) {
    std::cout << "tag ";
    std::cout << "id: " << this->id << "\t";
    std::cout << "detected: " << this->detected << "\t";
    std::cout << "position: ";
    std::cout << "(";
    std::cout << this->position(0) << ", ";
    std::cout << this->position(1) << ", ";
    std::cout << this->position(2);
    std::cout << ")";
    std::cout << std::endl;
  }
};

}  // namespace atl
#endif
