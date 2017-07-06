#ifndef ATL_GAZEBO_BRIDGE_DJI_QUADROTOR_NODE_HPP
#define ATL_GAZEBO_BRIDGE_DJI_QUADROTOR_NODE_HPP

#include <cmath>
#include <random>

#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <dji_sdk/AttitudeControl.h>
#include <dji_sdk/AttitudeControlRequest.h>
#include <dji_sdk/AttitudeControlResponse.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/Velocity.h>

#include "atl/gazebo/clients/quadrotor_gclient.hpp"
#include "atl/ros/utils/node.hpp"
#include "atl/utils/utils.hpp"

namespace atl {
namespace gazebo_bridge {

// NODE SETTINGS
#define NODE_NAME "atl_dji_quadrotor"
#define NODE_RATE 200

// PUBLISH TOPICS
#define DJI_LOCAL_POSITION_RTOPIC "/dji_sdk/local_position"
#define DJI_ATTITUDE_QUATERNION_RTOPIC "/dji_sdk/attitude_quaternion"
#define DJI_VELOCITY_RTOPIC "/dji_sdk/velocity"

// SUBSCRIBE TOPICS
#define DJI_ATTITUDE_SETPOINT_RTOPIC "/dji_sdk/attitude_control"

class DJIQuadrotorNode : public gaz::QuadrotorGClient, public ROSNode {
public:
  DJIQuadrotorNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(const std::string &node_name, int hz);
  void poseCallback(RPYPosePtr &msg);
  void velocityCallback(ConstVector3dPtr &msg);
  bool attitudeControlCallback(dji_sdk::AttitudeControl::Request &request,
                               dji_sdk::AttitudeControl::Response &response);
};

}  // namespace gazebo_bridge
}  // namespace atl
#endif
