#ifndef atl_GAZEBO_QUADROTOR_PLUGIN_HPP
#define atl_GAZEBO_QUADROTOR_PLUGIN_HPP

#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include "wave/utils/utils.hpp"
#include "wave/kinematics/quadrotor.hpp"
#include "atl_gazebo/gazebo_node.hpp"

namespace atl {
namespace gaz {

using namespace wave;

#define POSE_MSG atl_msgs::msgs::RPYPose
#define VELOCITY_MSG gazebo::msgs::Vector3d
#define ATT_SETPOINT_MSG atl_msgs::msgs::AttitudeSetpoint
#define POS_SETPOINT_MSG atl_msgs::msgs::PositionSetpoint
#define VEL_SETPOINT_MSG atl_msgs::msgs::VelocitySetpoint

#define POSE_GTOPIC "~/quadrotor/pose"
#define VELOCITY_GTOPIC "~/quadrotor/velocity"
#define ATT_SETPOINT_GTOPIC "~/quadrotor/setpoint/attitude"
#define POS_SETPOINT_GTOPIC "~/quadrotor/setpoint/position"
#define VEL_SETPOINT_GTOPIC "~/quadrotor/setpoint/velocity"

class QuadrotorGPlugin : public gazebo::ModelPlugin, public GazeboNode {
public:
  gazebo::physics::ModelPtr model;
  gazebo::event::ConnectionPtr update_conn;
  gazebo::common::Time prev_sim_time;

  ignition::math::Pose3d pose;
  gazebo::physics::LinkPtr body;
  QuadrotorModel quadrotor;

  QuadrotorGPlugin(void);
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
  void onUpdate(const gazebo::common::UpdateInfo &info);
  void simulate(double dt);
  void publishPose(void);
  void publishVelocity(void);
  void setAttitudeCallback(AttitudeSetpointPtr &msg);
  void setPositionCallback(PositionSetpointPtr &msg);
  void setVelocityCallback(VelocitySetpointPtr &msg);
};

}  // end of gaz namespace
}  // end of atl namespace
#endif
