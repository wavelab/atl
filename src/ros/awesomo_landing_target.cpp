#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include "awesomo/util.hpp"
#include "awesomo/camera.hpp"

#define ATIM_POSE_TOPIC "/atim/pose"
#define SENSOR_FUSION_POSE_TOPIC "/awesomo/landing_target/pose"


class LandingTarget
{
public:
    double mount_roll = 0;
    double mount_pitch = -M_PI/2;
    double mount_yaw = 0;

    tf::Quaternion apr_orientation;
    Eigen::Matrix3d mount_rot;
    Eigen::Vector3d t;
    Eigen::Matrix3d mirroring;

    void tfCallback( const geometry_msgs::PoseStamped &input);
    void subscribeToPose(void);
    void publishRotatedValues(int seq, ros::Time time);

    ros::NodeHandle n;
    ros::Subscriber poseSubscriber;
    ros::Publisher correction_publisher = n.advertise<geometry_msgs::PoseStamped>(
        SENSOR_FUSION_POSE_TOPIC,
        50
    );
};

void LandingTarget::tfCallback(const geometry_msgs::PoseStamped &input)
{
    Eigen::Vector3d temp;
    this->mirroring = Eigen::MatrixXd::Identity(3, 3);
    this->mirroring(0, 0) = -1;
    euler2RotationMatrix(
        this->mount_roll,
        this->mount_pitch,
        this->mount_yaw,
        this->mount_rot
    );
    temp << input.pose.position.x, input.pose.position.y, input.pose.position.z;
    this->t = this->mirroring * this->mount_rot * temp;

    // this->apr_orientation = input.pose.orientation;
}

void LandingTarget::subscribeToPose(void)
{
    ROS_INFO("subscribing to ATIM POSE");
    this->poseSubscriber = this->n.subscribe(
        ATIM_POSE_TOPIC,
        500,
        &LandingTarget::tfCallback,
        this
     );
}

void LandingTarget::publishRotatedValues(int seq, ros::Time time)
{
    geometry_msgs::PoseStamped correctedPose;

    correctedPose.header.seq = seq;
    correctedPose.header.stamp = time;
    correctedPose.header.frame_id = "corrected_camera_pose";
    correctedPose.pose.position.x = this->t[0];
    correctedPose.pose.position.y = this->t[1];
    correctedPose.pose.position.z = this->t[2];
    // correctedPose.pose.orientation = this->orientation;
    this->correction_publisher.publish(correctedPose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_test");
    ros::NodeHandle node_handle;
    ros::Rate rate(50.0);
    ros::Time last_request;
    LandingTarget *target;
    ROS_INFO("Running transform test");

    // setup
    target = new LandingTarget();
    target->subscribeToPose();
    int seq = 0;

    // publish
    while (ros::ok()){
        target->publishRotatedValues(seq, ros::Time::now());
        ros::spinOnce();
        rate.sleep();
        seq++;
    }

    return 0;
}
