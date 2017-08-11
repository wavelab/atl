#ifndef ATL_GAZEBO_BRIDGE_DF_CAMERA_NODE_HPP
#define ATL_GAZEBO_BRIDGE_DF_CAMERA_NODE_HPP

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>

#include "atl/gazebo/clients/df_camera_gclient.hpp"
#include "atl/ros/utils/node.hpp"
#include "atl/utils/math.hpp"

namespace atl {
namespace gazebo_bridge {

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
  int configure(int hz);
  void imageCallback(ConstImagePtr &msg);
};

} // namespace gazebo_bridge
} // namespace atl
#endif
