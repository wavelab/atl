#ifndef ATL_GAZEBO_BRIDGE_DJI_QUADROTOR_NODE_HPP
#define ATL_GAZEBO_BRIDGE_DJI_QUADROTOR_NODE_HPP

#include <cmath>
#include <random>

#include <Eigen/Geometry>

#include <ros/ros.h>

#include <atl/utils/utils.hpp>
#include <atl/ros/utils/node.hpp>
#include <atl/ros/utils/msgs.hpp>
#include "atl/gazebo/clients/quadrotor_gclient.hpp"

namespace atl {
namespace gazebo_bridge {

// NODE SETTINGS
#define NODE_RATE 200

// PUBLISH TOPICS
#define DJI_GPS_POSITION_RTOPIC "/dji_sdk/gps_position"
#define DJI_ATTITUDE_RTOPIC "/dji_sdk/attitude"
#define DJI_VELOCITY_RTOPIC "/dji_sdk/velocity"

// SUBSCRIBE TOPICS
#define DJI_SETPOINT_RTOPIC "/dji_sdk/flight_control_setpoint_generic"

/** DJI Quadrotor ROS Node */
class DJIQuadrotorNode : public gaz::QuadrotorGClient, public ROSNode {
public:
  // gps coordinates of uwaterloo
  double home_latitude = 43.472285;
  double home_longitude = -80.544858;
  double home_altitude = 333;

  DJIQuadrotorNode(int argc, char **argv) : ROSNode(argc, argv) {}

  /**
   * Configure
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
  void controlCallback(const sensor_msgs::Joy &msg);
};

} // namespace gazebo_bridge
} // namespace atl
#endif
