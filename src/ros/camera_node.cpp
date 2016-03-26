#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "awesomo/camera.hpp"


int main(int argc, char **argv)
{
    int seq;
    ros::init(argc, argv, "awesomo_camera");
    ros::NodeHandle n;
    ros::Rate rate(100);
    ros::Publisher publisher;
	geometry_msgs::PoseStamped pose;
	std::vector<AprilTagPose> pose_estimates;

    // setup
    seq = 0;
    publisher = n.advertise<geometry_msgs::PoseStamped>("awesomo/camera", 100);
    Camera cam(0, CAMERA_FIREFLY);
    cam.loadConfig(
        "default",
        "/home/chutsu/Dropbox/proj/awesomo/configs/pointgrey_firefly/ost.yaml"
    );
    cam.initCamera("default");

    while (ros::ok()) {
        // obtain poses
        pose_estimates = cam.step();

        // publish poses
        for (int i = 0; i < pose_estimates.size(); i++) {
            // pose header
            pose.header.stamp = ros::Time::now();
            pose.header.seq = seq;
            pose.header.frame_id = 1;

            // pose position
            pose.pose.position.x = pose_estimates[i].translation[0];
            pose.pose.position.y = pose_estimates[i].translation[1];
            pose.pose.position.z = pose_estimates[i].translation[2];

            // pose orientation
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 0.0;

            // publish and spin
            publisher.publish(pose);
        }

        // update
        seq++;

        ros::spinOnce();
    }

    return 0;
}
