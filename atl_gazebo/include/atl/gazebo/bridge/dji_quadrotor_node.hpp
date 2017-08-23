#ifndef ATL_GAZEBO_BRIDGE_DJI_QUADROTOR_NODE_HPP
#define ATL_GAZEBO_BRIDGE_DJI_QUADROTOR_NODE_HPP

#include <cmath>
#include <random>

#include <Eigen/Geometry>

#include <ros/ros.h>

#include <dji_sdk/dji_drone.h>

#include "atl/gazebo/clients/quadrotor_gclient.hpp"
#include <atl/ros/utils/msgs.hpp>
#include <atl/ros/utils/node.hpp>
#include <atl/utils/utils.hpp>

namespace atl {
namespace gazebo_bridge {

// NODE SETTINGS
#define NODE_NAME "atl_dji_quadrotor"
#define NODE_RATE 200

// PUBLISH TOPICS
#define DJI_GLOBAL_POSITION_RTOPIC "/dji_sdk/global_position"
#define DJI_LOCAL_POSITION_RTOPIC "/dji_sdk/local_position"
#define DJI_ATTITUDE_RTOPIC "/dji_sdk/attitude_quaternion"
#define DJI_VELOCITY_RTOPIC "/dji_sdk/velocity"

// SUBSCRIBE TOPICS
#define DJI_CONTROL_RTOPIC "/dji_sdk/attitude_control"

/** DJI Quadrotor ROS Node */
class DJIQuadrotorNode : public gaz::QuadrotorGClient, public ROSNode {
public:
  // gps coordinates of uwaterloo
  // double home_latitude = 43.472285;
  // double home_longitude = -80.544858;
  double home_latitude = 43.474024;
  double home_longitude = -80.540287;
  double home_altitude = 333;

  DJIQuadrotorNode(int argc, char **argv) : ROSNode(argc, argv) {}

  /**
   * Configure
   * @param node_name Name of ROS Node
   * @param hz ROS node rate in hertz
   */
  int configure(const int hz);

  /**
   * Quadrotor pose Gazebo callback
   * @param msg Roll, pitch and yaw message
   */
  void poseCallback(ConstPosePtr &msg);

  /**
   * Quadrotor velocity Gazebo callback
   * @param msg Velocity message in x, y, z
   */
  void velocityCallback(ConstVector3dPtr &msg);

  /**
   * Quadrotor control ROS callback
   *
   * Currently this function only supports a DJI control flag mode of 0x20.
   * This translates to being able to control the quadrotor's roll, pitch, yaw
   * and throttle
   *
   * @param msg Attitude setpoint for quadrotor
   */
  bool controlCallback(dji_sdk::AttitudeControl::Request &request,
                       dji_sdk::AttitudeControl::Response &response);
};

} // namespace gazebo_bridge
} // namespace atl
#endif
