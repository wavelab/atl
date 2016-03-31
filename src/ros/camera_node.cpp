#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "awesomo/camera.hpp"
#include "awesomo/util.hpp"

#define ROS_TOPIC "awesomo/apriltag_pose"
// #define ROS_TOPIC "mavros/vision_pose/pose"
// #define ROS_TOPIC "mavros/mocap/pose"
// #define ROS_TOPIC "mavros/vision_pose/pose_cov"

// #define FIREFLY_640 "/home/chutsu/Dropbox/proj/awesomo/configs/pointgrey_firefly/ost_640.yaml"
// #define FIREFLY_320 "/home/chutsu/Dropbox/proj/awesomo/configs/pointgrey_firefly/ost_320.yaml"
// #define FIREFLY_160 "/home/chutsu/Dropbox/proj/awesomo/configs/pointgrey_firefly/ost_160.yaml"

#define FIREFLY_640 "/home/odroid/awesomo/configs/pointgrey_firefly/ost_640.yaml"
#define FIREFLY_320 "/home/odroid/awesomo/configs/pointgrey_firefly/ost_320.yaml"
#define FIREFLY_160 "/home/odroid/awesomo/configs/pointgrey_firefly/ost_160.yaml"


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

void fix_coordinate_frames(
    TagPose &pose,
	double *rot_mat,
    double *pos,
    tf::Quaternion &quat
)
{
	double vec_pos[3];

    // translate from camera frame to ENU
    // x is times by -1 because april tag was in left-hand
    // co-ordinate frame commonly used by cameras
    pos[0] = pose.translation[1];
    pos[1] = -1 * pose.translation[2];
    pos[2] = pose.translation[0];
    // mat3_dot_vec3(rot_mat, pos, vec_pos);
    // pos[0] = vec_pos[0];
    // pos[1] = vec_pos[1];
    // pos[2] = vec_pos[2];
    // ROS_INFO("POSE X HERE IS %f", pose.translation[0]);
    // ROS_INFO("returned POSE X HERE IS %f", pos[0]);

    // convert euler angles to quaternions
    quat = euler2quat(pose.pitch, pose.roll, pose.yaw);
}

static void build_pose_stamped_msg(
    int seq,
    TagPose &pose,
	double *rot_mat,
    geometry_msgs::PoseStamped &pose_msg
)
{
	double pos[3];
	tf::Quaternion quat;

    // translate from camera frame to ENU
    fix_coordinate_frames(pose, rot_mat, pos, quat);

    // pose header
    pose_msg.header.seq = seq;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "pose_estimate";

    // pose position
    pose_msg.pose.position.x = pos[0];
    pose_msg.pose.position.y = pos[1];
    pose_msg.pose.position.z = pos[2];

    // pose orientation
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();
}

static void build_pose_stamped_cov_msg(
    int seq,
    TagPose &pose,
	double *rot_mat,
    geometry_msgs::PoseWithCovarianceStamped &pose_msg
)
{
	double pos[3];
	tf::Quaternion quat;

    // translate from camera frame to ENU
    fix_coordinate_frames(pose, rot_mat, pos, quat);

    // pose header
    pose_msg.header.seq = seq;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "pose_estimate";

    // pose position
    pose_msg.pose.pose.position.x = pos[0];
    pose_msg.pose.pose.position.y = pos[1];
    pose_msg.pose.pose.position.z = pos[2];

    // pose orientation
    pose_msg.pose.pose.orientation.x = quat.x();
    pose_msg.pose.pose.orientation.y = quat.y();
    pose_msg.pose.pose.orientation.z = quat.z();
    pose_msg.pose.pose.orientation.w = quat.w();

    // pose covariance
    pose_msg.pose.covariance[0] = 0.01;
    pose_msg.pose.covariance[7] = 0.01;
    pose_msg.pose.covariance[14] =0.01;
    pose_msg.pose.covariance[21] = 2000;
    pose_msg.pose.covariance[28] = 2000;
    pose_msg.pose.covariance[35] = 2000;
}


void print_pose_stamped_msg(geometry_msgs::PoseStamped &pose_msg)
{
    double roll;
    double pitch;
    double yaw;

    quat2euler(pose_msg.pose.orientation, &roll, &pitch, &yaw);

    ROS_INFO("x=%f ", pose_msg.pose.position.x);
    ROS_INFO("y=%f ", pose_msg.pose.position.y);
    ROS_INFO("z=%f ", pose_msg.pose.position.z);
    ROS_INFO("roll=%f ", rad2deg(roll));
    ROS_INFO("pitch=%f ", rad2deg(pitch));
    ROS_INFO("yaw=%f \n", rad2deg(yaw));
}

void print_pose_cov_stamped_msg(geometry_msgs::PoseWithCovarianceStamped &pose_msg)
{
    double roll;
    double pitch;
    double yaw;

    quat2euler(pose_msg.pose.pose.orientation, &roll, &pitch, &yaw);

    ROS_INFO("x=%f ", pose_msg.pose.pose.position.x);
    ROS_INFO("y=%f ", pose_msg.pose.pose.position.y);
    ROS_INFO("z=%f ", pose_msg.pose.pose.position.z);
    ROS_INFO("roll=%f ", rad2deg(roll));
    ROS_INFO("pitch=%f ", rad2deg(pitch));
    ROS_INFO("yaw=%f \n", rad2deg(yaw));
}

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

            print_pose_cov_stamped_msg(pose_msg);
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
