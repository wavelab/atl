#ifndef atl_GAZEBO_GIMBAL_PLUGIN_HPP
#define atl_GAZEBO_GIMBAL_PLUGIN_HPP

#include <boost/bind.hpp>

#include <gazebo/msgs/server_control.pb.h>
#include <gazebo/msgs/vector3d.pb.h>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/SensorManager.hh>

#include "atl/utils/math.hpp"

#include "atl/gazebo/gazebo_node.hpp"
#include "atl/gazebo/kinematics/gimbal.hpp"
#include "atl/gazebo/msgs/atl_msgs.hpp"
#include "atl/utils/utils.hpp"

namespace atl {
namespace gaz {

// MESSAGE TYPES
#define QUATERNION_MSG gazebo::msgs::Quaternion
#define ATT_SETPOINT_MSG gazebo::msgs::Vector3d

// PUBLISH TOPICS
#define FRAME_ORIENTATION_GTOPIC "~/gimbal/frame/orientation/inertial"
#define JOINT_ORIENTATION_GTOPIC "~/gimbal/joint/orientation/inertial"

// SUBSCRIBE GTOPICS
#define SETPOINT_GTOPIC "~/gimbal/joint/setpoint"
#define TRACK_GTOPIC "~/gimbal/target/track"

class GimbalGPlugin : public gazebo::ModelPlugin, public GazeboNode {
public:
  gazebo::physics::WorldPtr world;
  gazebo::physics::ModelPtr model;
  gazebo::physics::JointPtr roll_joint;
  gazebo::physics::JointPtr pitch_joint;
  gazebo::event::ConnectionPtr update_conn;
  gazebo::common::Time prev_sim_time;

  Gimbal2AxisModel gimbal;

  GimbalGPlugin(void);
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);
  void simulate(double dt);
  void onUpdate(const gazebo::common::UpdateInfo &info);
  void publishFrameOrientation(void);
  void publishJointOrientation(void);
  void setAttitudeCallback(ConstVector3dPtr &msg);
  void trackTargetCallback(ConstVector3dPtr &msg);
};

}  // namespace gaz
}  // namespace atl
#endif
