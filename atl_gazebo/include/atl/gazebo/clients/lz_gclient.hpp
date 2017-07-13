#ifndef atl_GAZEBO_LZ_CLIENT_HPP
#define atl_GAZEBO_LZ_CLIENT_HPP

#include <string>
#include <vector>

#include "atl/gazebo/gazebo_node.hpp"

namespace atl {
namespace gaz {

// PUBLISH TOPICS
#define POSITION_SET_GTOPIC "~/lz/position/set"
#define VELOCITY_SET_GTOPIC "~/lz/velocity/set"
#define ANGULAR_VEL_SET_GTOPIC "~/lz/angular_velocity/set"

// SUBSCRIBE TOPICS
#define POSE_GTOPIC "~/lz/pose"

class LZGClient : public GazeboNode {
public:
  bool connected;
  ignition::math::Pose3d pose;

  LZGClient();
  ~LZGClient();
  int configure();
  virtual void poseCallback(ConstPosePtr &msg);
  void setXYPosition(double x, double y);
  void setVelocity(double vel);
  void setAngularVelocity(double ang_vel);
};

}  // namespace gaz
}  // namespace atl
#endif
