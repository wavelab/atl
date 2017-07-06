#ifndef atl_GAZEBO_LZ_PLUGIN_HPP
#define atl_GAZEBO_LZ_PLUGIN_HPP

#include <math.h>

#include <boost/bind.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "atl/gazebo/gazebo_node.hpp"
#include "atl/gazebo/kinematics/two_wheel.hpp"


namespace atl {
namespace gaz {

// PUBLISH TOPICS
#define POSE_GTOPIC "~/lz/pose"

// SUBSCRIBE TOPICS
#define POSITION_SET_GTOPIC "~/lz/position/set"
#define VELOCITY_SET_GTOPIC "~/lz/velocity/set"
#define ANGULAR_VEL_SET_GTOPIC "~/lz/angular_velocity/set"

class LZGPlugin : public gazebo::ModelPlugin, public GazeboNode {
public:
  gazebo::physics::ModelPtr model;
  gazebo::event::ConnectionPtr update_conn;
  gazebo::common::Time prev_sim_time;

  Vec3 robot_states;
  Vec3 robot_inputs;

  LZGPlugin(void);
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
  void onUpdate(const gazebo::common::UpdateInfo &info);
  void publishPose(void);
  void positionCallback(ConstVector3dPtr &msg);
  void velocityCallback(ConstAnyPtr &msg);
  void angularVelocityCallback(ConstAnyPtr &msg);
};

}  // namespace gaz
}  // namespace atl
#endif
