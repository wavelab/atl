#ifndef WAVESIM_ROS_UTILS_UTILS_HPP
#define WAVESIM_ROS_UTILS_UTILS_HPP

#include "wave/utils/utils.hpp"

namespace wavesim {
namespace ros {

using namespace wave;

int ros2gaz(Vec3 ros, Vec3 &gaz);
int ros2gaz(Quaternion ros, Quaternion &gaz);
int gaz2ros(Vec3 gaz, Vec3 &ros);
int gaz2ros(Quaternion gaz, Quaternion &ros);

}  // end of ros namespace
}  // end of wavesim namespace
#endif
