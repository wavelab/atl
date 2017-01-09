#ifndef __AWESOMO_ROS_NODES_CAMERA_NODE_HPP__
#define __AWESOMO_ROS_NODES_CAMERA_NODE_HPP__

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/utils/node.hpp"
#include "awesomo_ros/utils/msgs.hpp"


namespace awesomo {

// NODE SETTINGS
#define CAMERA_NODE_NAME "awesomo_camera"
#define CAMERA_NODE_RATE 100

// PUBLISH TOPICS
#define CAMERA_IMAGE_TOPIC "/awesomo/camera/image"

// SUBSCRIBE TOPICS
#define SHUTDOWN_TOPIC "/awesomo/camera/shutdown"

class CameraNode : public ROSNode {
public:
  Camera camera;
  cv::Mat image;

  CameraNode(int argc, char **argv) : ROSNode(argc, argv) {}
  int configure(std::string node_name, int hz);
  int loopCallback(void);
  int publishImage(void);
};

}  // end of awesomo namespace
#endif
