#include <ros/ros.h>

#include "awesomo_core/vision/camera.hpp"


int main(int argc, char **argv) {
  int seq;
  int timeout;
  std::string camera_config_path;

  ros::init(argc, argv, "awesomo_camera");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);

  // setup
  timeout = 0;
  n.getParam("/camera_config_path", camera_config_path);
  Camera cam(camera_config_path);

  while (ros::ok()) {
    // obtain image
    cam.step(timeout);

    // publish and spin
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
