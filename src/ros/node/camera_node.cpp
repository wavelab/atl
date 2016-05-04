#include "awesomo/ros/node/camera_node.hpp"


int main(int argc, char **argv)
{
    int seq;
    int timeout;
	double rot_mat[9];
	TagPose pose;
	std::vector<TagPose> pose_estimates;
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    std::string camera_config_path;

    ros::init(argc, argv, "awesomo_camera");
    ros::NodeHandle n;
    ros::Rate rate(100);
    ros::Publisher publisher;

    // setup
    seq = 0;
    timeout = 0;
    rotation_matrix(M_PI_2, -M_PI_2, 0.0, rot_mat);

    // ROS specifics
    publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(ROS_TOPIC, 100);

    // camera specifics
	n.getParam("/camera_config_path", camera_config_path);
    Camera cam(camera_config_path);
    ROS_INFO("Camera node is publishing pose data!");


    // ROS node loop
    while (ros::ok()) {
        // obtain pose estimates from camera
        pose_estimates = cam.step(timeout);

        // publish poses
        for (int i = 0; i < pose_estimates.size(); i++) {
            // build pose message
            pose = pose_estimates[i];
            build_pose_stamped_cov_msg(seq, pose, rot_mat, pose_msg);
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
