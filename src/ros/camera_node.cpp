#include "awesomo/ros/camera_node.hpp"


int main(int argc, char **argv)
{
    int seq;
    int timeout;
	double rot_mat[9];
	TagPose pose;
	std::vector<TagPose> pose_estimates;
    // geometry_msgs::PoseWithCovariance pose_msg;
    geometry_msgs::PoseWithCovarianceStamped pose_msg;

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

            // publish and spin
            publisher.publish(pose_msg);
            seq++;

            // print_pose_cov_stamped_msg(pose_msg);
        }

        // send last known estimate if tag not detected
        if (pose_estimates.size() == 0) {
             // ROS_INFO("Still publishing!");

            // publish and spin
            pose_msg.header.seq = seq;
            pose_msg.header.stamp = ros::Time::now();
            publisher.publish(pose_msg);
            // update
            seq++;
        }

        // sleep
        rate.sleep();
    }
    ROS_INFO("Camera node exited!");

    return 0;
}
