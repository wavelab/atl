#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "awesomo/camera.hpp"


// YPR
void rotation_matrix(double phi, double theta, double psi, double *rot_mat)
{
    //  rotation matrix
    rot_mat[0] = cos(phi) * cos(theta);
    rot_mat[1] = sin(phi) * cos(theta);
    rot_mat[2] = -1 * sin(theta);

    rot_mat[3] = cos(phi) * sin(theta) * sin(psi) - cos(psi) * sin(phi);
    rot_mat[4] = cos(phi) * cos(psi) + sin(phi) * sin(theta) * sin(psi);
    rot_mat[5] = cos(theta) * sin(psi);

    rot_mat[6] = sin(phi) * sin(psi) + cos(phi) * cos(psi) * sin(theta);
    rot_mat[7] = cos(psi) * sin(phi) * sin(theta) - cos(phi) * sin(psi);
    rot_mat[8] = cos(theta) * cos(psi);
}

void mat3_dot_vec3(double *m, double *v, double *out)
{
    out[0] = m[0] * v[0] + m[3] * v[1] + m[6] * v[2];
    out[1] = m[1] * v[0] + m[4] * v[1] + m[7] * v[2];
    out[2] = m[2] * v[0] + m[5] * v[1] + m[8] * v[2];
}

int main(int argc, char **argv)
{
    int seq;
    int timeout;
    ros::init(argc, argv, "awesomo_camera");
    ros::NodeHandle n;
    ros::Rate rate(100);
    ros::Publisher publisher;
	geometry_msgs::PoseStamped pose;
	std::vector<AprilTagPose> pose_estimates;
	double rot_mat[9];
	double vec_pos[3];
	double pos[3];

    // setup
    seq = 0;
    timeout = 0;
    // create rotation matrix - YAW (90) and PITCH (-90)
    rotation_matrix(M_PI_2, -M_PI_2, 0.0, rot_mat);
    // publisher = n.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 100);
    publisher = n.advertise<geometry_msgs::PoseStamped>("mavros/mocap/pose", 100);
    Camera cam(0, CAMERA_FIREFLY);
    cam.loadConfig(
        "default",
        // "/home/chutsu/Dropbox/proj/awesomo/configs/pointgrey_firefly/ost_640.yaml"
        "/home/odroid/awesomo/configs/pointgrey_firefly/ost_640.yaml"
    );
    cam.loadConfig(
        "320",
        // "/home/chutsu/Dropbox/proj/awesomo/configs/pointgrey_firefly/ost_320.yaml"
        "/home/odroid/awesomo/configs/pointgrey_firefly/ost_320.yaml"
    );
    cam.loadConfig(
        "160",
        // "/home/chutsu/Dropbox/proj/awesomo/configs/pointgrey_firefly/ost_160.yaml"
        "/home/odroid/awesomo/configs/pointgrey_firefly/ost_160.yaml"
    );
    // cam.initCamera("default");
    cam.initCamera("320");
    // cam.initCamera("160");

    while (ros::ok()) {
        // obtain poses
        pose_estimates = cam.step(timeout);

        // publish poses
        for (int i = 0; i < pose_estimates.size(); i++) {
            // pose header
            pose.header.stamp = ros::Time::now();
            pose.header.seq = seq;
            pose.header.frame_id = 1;

            vec_pos[0] = pose_estimates[i].translation[0];
            vec_pos[1] = pose_estimates[i].translation[1];
            vec_pos[2] = pose_estimates[i].translation[2];
            mat3_dot_vec3(rot_mat, vec_pos, pos);

            // pose position
            // x is times by -1 because april tag was in left-hand
            // co-ordinate frame commonly used by cameras
            pose.pose.position.x = -1 * pos[0];
            pose.pose.position.y = pos[1];
            pose.pose.position.z = pos[2];

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
