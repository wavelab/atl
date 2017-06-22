#ifndef WAVESIM_GAZEBO_LZ_CLIENT_HPP
#define WAVESIM_GAZEBO_LZ_CLIENT_HPP

#include <string>
#include <vector>

#include "wavesim_gazebo/gazebo_node.hpp"

namespace wavesim {
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

  LZGClient(void);
  ~LZGClient(void);
  int configure(void);
  virtual void poseCallback(ConstPosePtr &msg);
  void setXYPosition(double x, double y);
  void setVelocity(double vel);
  void setAngularVelocity(double ang_vel);
};

}  // end of gaz namespace
}  // end of wavesim namespace
#endif
