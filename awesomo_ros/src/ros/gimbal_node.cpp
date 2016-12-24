#include "awesomo_core/vision/ros/atim.hpp"


int main(int argc, char **argv) {
  int seq;
  int timeout;
  TagPose pose;
  std::vector<TagPose> pose_estimates;
  awesomo_msgs::TagPoseStamped pose_msg;
  std::string camera_config_path;
  std::string gimbal_config_path;
  ros::Time last_request;
  float dt;

  ros::init(argc, argv, "awesomo_camera");
  ros::NodeHandle n;
  ros::Rate rate(100);
  ros::Publisher publisher;

  // setup
  seq = 0;
  timeout = 0;

  // ROS specifics
  publisher = n.advertise<awesomo_msgs::TagPoseStamped>(ROS_TOPIC, 100);

  // camera specifics
  n.getParam("/camera_config_path", camera_config_path);
  n.getParam("/gimbal_config_path", gimbal_config_path);
  Camera cam(camera_config_path);
  cam.initGimbal(gimbal_config_path);
  ROS_INFO("Camera node is publishing pose data!");

  // ROS node loop
  last_request = ros::Time::now();
  while (ros::ok()) {
    // obtain pose estimates from camera
    dt = (ros::Time::now() - last_request).toSec();
    pose_estimates = cam.step(timeout, dt);

    // publish poses
    for (int i = 0; i < pose_estimates.size(); i++) {
      // build pose message
      pose = pose_estimates[i];
      build_pose_stamped_msg(seq, pose, pose_msg);
      pose_msg.tag_detected = 1;
      publisher.publish(pose_msg);

      // update
      seq++;
    }

    // not sure we want to do this in the final version?
    // send last known estimate if tag not detected
    if (pose_estimates.size() == 0) {
      // publish and spin
      pose_msg.header.seq = seq;
      pose_msg.header.stamp = ros::Time::now();
      pose_msg.tag_detected = 0;
      publisher.publish(pose_msg);

      // update
      seq++;
    }
    // sleep
    rate.sleep();
    ros::spinOnce();
  }
  ROS_INFO("Camera node exited!");

  return 0;
}
