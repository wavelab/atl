#ifndef WAVESIM_GAZEBO_LZ_PLUGIN_HPP
#define WAVESIM_GAZEBO_LZ_PLUGIN_HPP

#include <math.h>

#include <boost/bind.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "wave/kinematics/twowheel.hpp"
#include "wavesim_gazebo/gazebo_node.hpp"


namespace wavesim {
namespace gaz {

using namespace wave;

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

}  // end of gaz namespace
}  // end of wavesim namespace
#endif
