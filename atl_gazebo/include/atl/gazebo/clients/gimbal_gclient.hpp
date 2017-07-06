#ifndef atl_GAZEBO_GIMBAL_CLIENT_HPP
#define atl_GAZEBO_GIMBAL_CLIENT_HPP

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "atl/utils/math.hpp"
#include "atl/gazebo/gazebo_node.hpp"

namespace atl {
namespace gaz {

// MESSAGE TYPES
#define VEC3_MSG gazebo::msgs::Vector3d

// PUBLISH TOPICS
#define SETPOINT_GTOPIC "~/gimbal/joint/setpoint"
#define TRACK_GTOPIC "~/gimbal/target/track"

// SUBSCRIBE TOPICS
#define FRAME_ORIENTATION_GTOPIC "~/gimbal/frame/orientation/inertial"
#define JOINT_ORIENTATION_GTOPIC "~/gimbal/joint/orientation/inertial"

class GimbalGClient : public GazeboNode {
public:
  bool connected;
  Quaternion frame_orientation;
  Quaternion joint_orientation;

  GimbalGClient(void);
  ~GimbalGClient(void);
  int configure(void);
  void setAttitude(Vec3 euler_if);
  void trackTarget(Vec3 target_cf);
  virtual void frameOrientationCallback(ConstQuaternionPtr &msg);
  virtual void jointOrientationCallback(ConstQuaternionPtr &msg);
};

}  // namespace gaz
}  // namespace atl
#endif
