#include <std_msgs/Float64.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <atim/AtimPoseStamped.h>

#include "awesomo/util.hpp"
#include "awesomo/camera.hpp"

#define ATIM_POSE_TOPIC "/atim/pose"
#define ATIM_REDIVERT_POSE_TOPIC "/atim/pose/standard"
#define LANDING_TARGET_TOPIC "/awesomo/landing_target/pose"
#define MAVROS_LOCAL_POSITION_TOPIC "/mavros/local_position/pose"

class LandingTarget
{
public:
    CameraMountRBT cam_rbt;
    LandingTargetPosition landing_target_position;
    Pose FC_local_pose; // local pose from Flight Controller, FC

    LandingTarget(CameraMountRBT &cam_rbt);
    void localPoseCallback(const geometry_msgs::PoseStamped &input);
    void cameraRBTCallback(const atim::AtimPoseStamped &input);
    void cameraRBTWithIMUCallback(const atim::AtimPoseStamped &msg);
    void subscribeToAtimPose(void);
    void subscribeToLocalPositionPose(void);
    void publishRotatedValues(int seq, ros::Time time);

    ros::NodeHandle n;
    ros::Subscriber atimPoseSubscriber;
    ros::Subscriber localPositionPoseSubscriber;
    ros::Publisher correction_publisher;
    ros::Publisher redivert_publisher;

    tf::TransformBroadcaster body_planar_tf_br;
    tf::Transform body_planar_tf;
    tf::TransformBroadcaster body_planar_target_location_br;
    tf::Transform body_planner_target_location_tf;

};

LandingTarget::LandingTarget(CameraMountRBT &cam_rbt)
{
    this->cam_rbt = cam_rbt;
    this->FC_local_pose.roll = 0.0;
    this->FC_local_pose.pitch = 0.0;
    this->FC_local_pose.yaw = 0.0;
    this->landing_target_position.x = 0.0;
    this->landing_target_position.y = 0.0;
    this->landing_target_position.z = 0.0;
    this->landing_target_position.detected = false;
    this->correction_publisher = n.advertise<atim::AtimPoseStamped>(
        LANDING_TARGET_TOPIC,
        50
    );
    this->redivert_publisher = n.advertise<geometry_msgs::PoseStamped>(
        ATIM_REDIVERT_POSE_TOPIC,
        50
    );
}

void LandingTarget::cameraRBTCallback(const atim::AtimPoseStamped &msg)
{
    bool tag_detected;

    if (msg.tag_detected) {
        this->landing_target_position.x = msg.pose.position.x;
        this->landing_target_position.y = -1 * msg.pose.position.y; // atim frame weirdness
        this->landing_target_position.z = msg.pose.position.z;
        this->landing_target_position.detected = msg.tag_detected;

        this->cam_rbt.applyRBTtoPosition(this->landing_target_position);
        // negate the rotation using the imu
        applyRotationToPosition(
            this->FC_local_pose.roll,
            this->FC_local_pose.pitch * -1,
            this->FC_local_pose.yaw * 0, // do not correct for yaw (body planar frame)
            this->landing_target_position
        );
        ROS_INFO(
            "target pose in NED, body planar: %f\t%f\t%f",
            this->landing_target_position.x,
            this->landing_target_position.y,
            this->landing_target_position.z
        );

        // Apply translation required due to camera mount
        // in NED
        this->landing_target_position.x += -0.07;
        this->landing_target_position.y += 0.00;
        this->landing_target_position.z += 0.08;

        this->body_planner_target_location_tf.setOrigin(
            tf::Vector3(
                this->landing_target_position.x,
                this->landing_target_position.y,
                this->landing_target_position.z
            )
        );

        this->body_planner_target_location_tf.setRotation(
            tf::Quaternion(0, 0, 0, 1) // identity Quaternion
       );

        this->body_planar_target_location_br.sendTransform(
            tf::StampedTransform(
                this->body_planner_target_location_tf,
                ros::Time::now(),
                "body_planar_frame",
                "target_location"
            )
        );
    }

    else{
        this->landing_target_position.detected = msg.tag_detected;
    }
}


void LandingTarget::localPoseCallback(const geometry_msgs::PoseStamped &input)
{
    this->FC_local_pose.x = input.pose.position.x;
    this->FC_local_pose.y = input.pose.position.y;
    this->FC_local_pose.z = input.pose.position.z;

    quat2euler(
        input.pose.orientation,
        &this->FC_local_pose.roll,
        &this->FC_local_pose.pitch,
        &this->FC_local_pose.yaw
    );

    tf::Quaternion q;
    q.setRPY(
        this->FC_local_pose.roll,
        this->FC_local_pose.pitch,
        0 * this->FC_local_pose.yaw
    );

    // rotate around y, then  z
    tf::Quaternion q_enu_to_ned;
    q_enu_to_ned.setRPY(0, M_PI, M_PI);

    this->body_planar_tf.setOrigin(tf::Vector3(0, 0, 0));
    this->body_planar_tf.setRotation(q.inverse() * q_enu_to_ned);

    this->body_planar_tf_br.sendTransform(
        tf::StampedTransform(
            this->body_planar_tf, ros::Time::now(), "pixhawk", "body_planar_frame"
        )
    );
}

void LandingTarget::subscribeToAtimPose(void)
{
    ROS_INFO("subscribing to ATIM POSE");
    this->atimPoseSubscriber = this->n.subscribe(
        ATIM_POSE_TOPIC,
        50,
        &LandingTarget::cameraRBTCallback,
        this
    );
}

void LandingTarget::subscribeToLocalPositionPose(void)
{
    ROS_INFO("subscribing to MAVROS LOCAL POSITION POSE");
    this->localPositionPoseSubscriber = this->n.subscribe(
        MAVROS_LOCAL_POSITION_TOPIC,
        50,
        &LandingTarget::localPoseCallback,
        this
     );
}

void LandingTarget::publishRotatedValues(int seq, ros::Time time)
{
    atim::AtimPoseStamped correctedPose;
    geometry_msgs::PoseStamped correctedPose2;

    correctedPose.header.seq = seq;
    correctedPose.header.stamp = time;
    correctedPose.header.frame_id = "awesomo/tracker_position";
    correctedPose.pose.position.x = this->landing_target_position.x;
    correctedPose.pose.position.y = this->landing_target_position.y;
    correctedPose.pose.position.z = this->landing_target_position.z;
    correctedPose.tag_detected = this->landing_target_position.detected;

    correctedPose2.header.seq = seq;
    correctedPose2.header.stamp = time;
    correctedPose2.pose.position.x = this->landing_target_position.x;
    correctedPose2.pose.position.y = this->landing_target_position.y;
    correctedPose2.pose.position.z = this->landing_target_position.z;

    this->correction_publisher.publish(correctedPose);
    this->redivert_publisher.publish(correctedPose2);
}

int main(int argc, char **argv)
{
    // setup
    ros::init(argc, argv, "awesomo_tracker");
    ros::NodeHandle node_handle;
    ros::Rate rate(50.0);
    ros::Time last_request;

    double camera_mount_roll = 0;
    double camera_mount_pitch = -M_PI/2;
    double camera_mount_yaw = 0;

    CameraMountRBT cam_rbt;
    LandingTarget *target;

    // do not add translations here, they are not applied properly..
    cam_rbt.initialize(
        camera_mount_roll,
        camera_mount_pitch,
        camera_mount_yaw,
        0,
        0,
        0
    );

    cam_rbt.initializeMirrorMtx(-1, 1, 1);
    target = new LandingTarget(cam_rbt);

    ROS_INFO("Publishing tracker");
    target->subscribeToAtimPose();
    target->subscribeToLocalPositionPose();
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
