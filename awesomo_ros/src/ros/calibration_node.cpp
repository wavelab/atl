#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "awesomo_core/vision/camera.hpp"


int main(int argc, char **argv) {
  int seq;
  cv::Mat image;
  cv::Mat result;
  sensor_msgs::ImagePtr msg;
  std::string camera_config_path;

  ros::init(argc, argv, "awesomo_camera");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("awesomo/camera/image", 30);

  // setup
  n.getParam("/camera_config_path", camera_config_path);
  Camera cam(camera_config_path);

  while (ros::ok()) {
    // obtain image
    cam.getFrame(image);
    cv::resize(image, result, cv::Size(640 / 1, 480 / 1));
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", result).toImageMsg();
    cv::imshow("the image", image);
    cv::waitKey(1);

    // publish and spin
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
