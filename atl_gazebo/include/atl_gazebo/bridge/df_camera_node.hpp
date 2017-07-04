#ifndef ATL_GAZEBO_BRIDGE_DF_CAMERA_NODE_HPP
#define ATL_GAZEBO_BRIDGE_DF_CAMERA_NODE_HPP

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>

#include "atl/utils/math.hpp"
#include "atl/ros/utils/node.hpp"
#include "atl_gazebo/clients/df_camera_gclient.hpp"

namespace atl {
namespace ros {

// NODE SETTINGS
#define NODE_NAME "atl_df_camera"
#define NODE_RATE 100

// PUBLISH TOPICS
#define CAMERA_IMAGE_RTOPIC "/atl/df_camera/image"

class DFCameraNode : public gaz::DFCameraGClient, public ROSNode {
public:
  bool configured;

  DFCameraNode(int argc, char **argv) : ROSNode(argc, argv) {
    this->configured = false;
  }
  int configure(const std::string &node_name, int hz);
  void imageCallback(ConstImagePtr &msg);
};

}  // namespace ros
}  // namespace atl
#endif
