#ifndef ATL_ROS_UTILS_UTILS_HPP
#define ATL_ROS_UTILS_UTILS_HPP

#include "atl/utils/utils.hpp"

namespace atl {
namespace ros {

int ros2gaz(Vec3 ros, Vec3 &gaz);
int ros2gaz(Quaternion ros, Quaternion &gaz);
int gaz2ros(Vec3 gaz, Vec3 &ros);
int gaz2ros(Quaternion gaz, Quaternion &ros);

}  // namespace ros
}  // namespace atl
#endif
