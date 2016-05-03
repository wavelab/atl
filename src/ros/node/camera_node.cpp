#include "awesomo/ros/node/camera_node.hpp"

#define MOCAP_TOPIC "/awesomo/mocap/pose"
#include <std_msgs/Float64.h>

float pos_x;
float pos_y;
float pos_z;
float quat_x;
float quat_y;
float quat_z;
float quat_w;


void mocapCallback(const geometry_msgs::PoseStamped &msg)
{
    pos_x = msg.pose.position.x;
    pos_y = msg.pose.position.y;
    pos_z = msg.pose.position.z;

    quat_x = msg.pose.orientation.x;
    quat_y = msg.pose.orientation.y;
    quat_z = msg.pose.orientation.z;
    quat_w = msg.pose.orientation.w;
}

int main(int argc, char **argv)
{
    int seq;
    int timeout;
	double rot_mat[9];
	TagPose pose;
	std::vector<TagPose> pose_estimates;
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    std_msgs::Float64 stupid;

    ros::init(argc, argv, "awesomo_camera");
    ros::NodeHandle n;
    ros::Rate rate(100);
    ros::Publisher publisher;

    // setup
    seq = 0;
    timeout = 0;
    rotation_matrix(M_PI_2, -M_PI_2, 0.0, rot_mat);

    // ROS specifics
    // publisher = n.advertise<geometry_msgs::PoseStamped>(ROS_TOPIC, 100);
    publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(ROS_TOPIC, 100);

    // camera specifics
    Camera cam(0, CAMERA_FIREFLY);
    cam.loadConfig("default", FIREFLY_640);
    cam.loadConfig("320", FIREFLY_320);
    cam.loadConfig("160", FIREFLY_160);
    cam.initCamera("320");
    ROS_INFO("Camera node is publishing pose data!");


    // ROS node loop
    while (ros::ok()) {
        pose_estimates = cam.step(timeout);

        // publish poses
        for (int i = 0; i < pose_estimates.size(); i++) {
            // build pose message
            pose = pose_estimates[i];
            build_pose_stamped_cov_msg(seq, pose, rot_mat, pose_msg);
            publisher.publish(pose_msg);
            seq++;
        }


        // not sure we want to do this in the final version?
        // send last known estimate if tag not detected
        if (pose_estimates.size() == 0) {
            // publish and spin
            pose_msg.header.seq = seq;
            pose_msg.header.stamp = ros::Time::now();
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
