#ifndef __AWESOMO_VISION_APRILTAG_DATA_HPP__
#define __AWESOMO_VISION_APRILTAG_DATA_HPP__

#include "awesomo_core/utils/utils.hpp"

namespace awesomo {

class TagPose {
public:
  int id;
  bool detected;
  Vec3 position;

  TagPose(void) {
    this->id = -1;
    this->detected = false;
    this->position << 0.0, 0.0, 0.0;
  };

  TagPose(int id, bool detected, Vec3 pos) {
    this->id = id;
    this->detected = detected;
    this->position = pos;
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

}  // end of awesomo namespace
#endif
