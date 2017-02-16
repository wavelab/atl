#ifndef __AWESOMO_ROS_NODES_IMU_NODE_HPP__
#define __AWESOMO_ROS_NODES_IMU_NODE_HPP__

#include <ros/ros.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"


namespace awesomo {

// NODE SETTINGS
#define NODE_NAME "awesomo_imu"
#define NODE_RATE 200

// PUBLISH TOPICS
#define IMU_TOPIC "/awesomo/imu"
#define FRAME_ORIENTATION_TOPIC "/awesomo/gimbal/frame/orientation/inertial"
#define JOINT_ORIENTATION_TOPIC "/awesomo/gimbal/joint/orientation/inertial"
#define POSITION_TOPIC "/awesomo/gimbal/position/inertial"

// SUBSCRIBE TOPICS
#define QUAD_POSE_TOPIC "/mavros/local_position/pose"

class IMUNode : public ROSNode {
public:
  MPU6050 imu;
  std::string quad_frame;

  IMUNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(std::string node_name, int hz);
  int publishIMU(Vec3 euler);
  int publishFrameOrientation(Quaternion q);
  int publishJointOrientation(Quaternion q);
  int publishPosition(Vec3 pos);
  void quadPoseCallback(const geometry_msgs::PoseStamped &msg);
  int loopCallback(void);
};

}  // end of awesomo namespace
#endif
