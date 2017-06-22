#ifndef WAVESIM_ROS_NODES_DJI_QUADROTOR_NODE_HPP
#define WAVESIM_ROS_NODES_DJI_QUADROTOR_NODE_HPP

#include <cmath>
#include <random>

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <dji_sdk/AttitudeControl.h>
#include <dji_sdk/AttitudeControlRequest.h>
#include <dji_sdk/AttitudeControlResponse.h>
#include <dji_sdk/LocalPosition.h>
#include <dji_sdk/AttitudeQuaternion.h>
#include <dji_sdk/Velocity.h>

#include "wave/utils/utils.hpp"
#include "wavesim_ros/utils/node.hpp"
#include "wavesim_ros/utils/utils.hpp"
#include "wavesim_gazebo/clients/quadrotor_gclient.hpp"


namespace wavesim {
namespace ros {

// NODE SETTINGS
#define NODE_NAME "wavesim_dji_quadrotor"
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

}  // end of ros namespace
}  // end of wavesim namespace
#endif
