#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include "awesomo/camera.hpp"
#include "awesomo/util.hpp"

#define POSE_TOPIC "/atim/pose"

class CameraPoseCorrection
{
    public:
        CameraMountRBT cam_rbt;

        tf::Quaternion apr_orientation;
        Eigen::Matrix3d mount_rot;
        // Eigen::Vector3d t;
        Position position;
        Eigen::Matrix3d mirroring;

        CameraPoseCorrection(CameraMountRBT &cam_rbt);
        void callback( const geometry_msgs::PoseStamped &input);
        void subscribeToPose(void);
        void publishRotatedValues(int seq, ros::Time time);

        ros::NodeHandle n;
        ros::Subscriber poseSubscriber;
        ros::Publisher correction_publisher = n.advertise<geometry_msgs::PoseStamped>(
                "awesomo/tracker/position",
                50
                );
};

CameraPoseCorrection::CameraPoseCorrection(CameraMountRBT &cam_rbt)
{
    this->cam_rbt = cam_rbt;
}

void CameraPoseCorrection::callback(const geometry_msgs::PoseStamped &input)
{
    // Eigen::Vector3d temp;
    // temp << input.pose.position.x, input.pose.position.y, input.pose.position.z;
    this->position.x = input.pose.position.x;
    this->position.y = input.pose.position.y;
    this->position.z = input.pose.position.z;

    this->cam_rbt.applyRBTtoPosition(this->position);
    // this->t = this->mirroring * this->mount_rot * temp;
}

void CameraPoseCorrection::subscribeToPose(void)
{
    ROS_INFO("subscribing to POSE");
    this->poseSubscriber = this->n.subscribe(
            POSE_TOPIC,
            500,
            &CameraPoseCorrection::callback,
            this
     );
}


void CameraPoseCorrection::publishRotatedValues(int seq, ros::Time time)
{
    geometry_msgs::PoseStamped correctedPose;

    correctedPose.header.seq = seq;
    correctedPose.header.stamp = time;
    correctedPose.header.frame_id = "awesomo/tracker_position";
    correctedPose.pose.position.x = this->position.x;
    correctedPose.pose.position.y = this->position.y;
    correctedPose.pose.position.z = this->position.z;
    this->correction_publisher.publish(correctedPose);
}

int main(int argc, char **argv)
{
    // setup
    ros::init(argc, argv, "awesomo_tracker");
    ros::NodeHandle node_handle;
    ros::Rate rate(50.0);
    ros::Time last_request;

    double mount_roll = 0;
    double mount_pitch = -M_PI/2;
    double mount_yaw = 0;
    double mount_x = 0.067;
    double mount_y = 0.0;
    double mount_z = 0.07;

    CameraMountRBT cam_rbt;
    CameraPoseCorrection *CPC;

    cam_rbt.initialize(
        mount_roll,
        mount_pitch,
        mount_yaw,
        mount_x,
        mount_y,
        mount_z
        );

    cam_rbt.initializeMirrorMtx(-1, 1, 1);
    CPC = new CameraPoseCorrection(cam_rbt);

    ROS_INFO("Publishing tracker");
    CPC->subscribeToPose();
    int seq = 0;
    while (ros::ok()){
        CPC->publishRotatedValues(seq, ros::Time::now());
        ros::spinOnce();
        rate.sleep();
        seq++;
    }

    return 0;
}



