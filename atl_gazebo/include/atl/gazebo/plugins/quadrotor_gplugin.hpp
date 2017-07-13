#ifndef atl_GAZEBO_QUADROTOR_PLUGIN_HPP
#define atl_GAZEBO_QUADROTOR_PLUGIN_HPP

#include <boost/bind.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "atl/gazebo/gazebo_node.hpp"
#include "atl/gazebo/kinematics/quadrotor.hpp"
#include "atl/utils/utils.hpp"

namespace atl {
namespace gaz {

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

  QuadrotorGPlugin();
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
  void onUpdate(const gazebo::common::UpdateInfo &info);
  void simulate(double dt);
  void publishPose();
  void publishVelocity();
  void setAttitudeCallback(AttitudeSetpointPtr &msg);
  void setPositionCallback(PositionSetpointPtr &msg);
  void setVelocityCallback(VelocitySetpointPtr &msg);
};

}  // namespace gaz
}  // namespace atl
#endif
