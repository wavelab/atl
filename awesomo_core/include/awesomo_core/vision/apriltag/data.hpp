#ifndef __AWESOMO_VISION_APRILTAG_DATA_HPP__
#define __AWESOMO_VISION_APRILTAG_DATA_HPP__

#include "awesomo_core/utils/utils.hpp"

namespace awesomo {

class TagPose {
public:
  int id;
  bool detected;
  Eigen::Vector3d position;

  TagPose(void) {
    this->id = -1;
    this->detected = false;
    this->position << 0.0, 0.0, 0.0;
  };
  TagPose(int id, bool detected, Eigen::Vector3d position) {
    this->id = id;
    this->detected = detected;
    this->position = position;
  }
};

}  // end of awesomo namespace
#endif
