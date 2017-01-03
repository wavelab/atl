#ifndef __AWESOMO_ROS_CAMERA_NODE_HPP__
#define __AWESOMO_ROS_CAMERA_NODE_HPP__

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include <awesomo_core/awesomo_core.hpp>

#include "awesomo_ros/ros_node.hpp"
#include "awesomo_ros/ros_msgs.hpp"


namespace awesomo {

#define CAMERA_NODE_NAME "awesomo_camera"
#define CAMERA_NODE_RATE 100

class CameraNode : public ROSNode {
public:
  Camera camera;
  cv::Mat image;

  int configure(std::string node_name, int hz);
  int loopCallback(void);
  int publishImage(void);
};

}  // end of awesomo namespace
#endif
