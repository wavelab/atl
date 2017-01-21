#ifndef __AWESOMO_ROS_NODES_GIMBAL_NODE_HPP__
#define __AWESOMO_ROS_NODES_GIMBAL_NODE_HPP__

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"

namespace awesomo {

// NODE SETTINGS
#define NODE_NAME "awesomo_gimbal"
#define NODE_RATE 100

// PUBLISH TOPICS
#define FRAME_RPY_TOPIC "/awesomo/gimbal/frame/orientiation/inertial"
#define CAMERA_IMU_TOPIC "/awesomo/gimbal/joint/imu"

// SUBSCRIBE TOPICS
#define SET_ATTITUDE_TOPIC "/awesomo/gimbal/setpoint/attitude"
#define SHUTDOWN_TOPIC "/awesomo/gimbal/shutdown"


class GimbalNode : public ROSNode {
public:
  Gimbal gimbal;

  GimbalNode(int argc, char **argv) : ROSNode(argc, argv) {}
  ~GimbalNode(void);
  int configure(std::string node_name, int hz);

  void setAttitudeCallback(const geometry_msgs::Vector3 &msg);
  int loopCallback(void);
  int seq;
};

}  // end of awesomo namespace
#endif
