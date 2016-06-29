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
    LandingTargetPosition position;
    Pose local_pose;

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

    tf::TransformBroadcaster landing_target_tf_broadcaster;
};

LandingTarget::LandingTarget(CameraMountRBT &cam_rbt)
{
    this->cam_rbt = cam_rbt;
    this->local_pose.roll = 0.0;
    this->local_pose.pitch = 0.0;
    this->local_pose.yaw = 0.0;
    this->position.x = 0.0;
    this->position.y = 0.0;
    this->position.z = 0.0;
    this->position.detected = false;
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
        this->position.x = msg.pose.position.x;
        this->position.y = msg.pose.position.y;
        this->position.z = msg.pose.position.z;
        this->position.detected = msg.tag_detected;

    // ROS_INFO(
    //     "pose before: %f\t%f\t%f",
    //     this->position.x,
    //     this->position.y,
    //     this->position.z
    // );

    // this->cam_rbt.applyRBTtoPosition(this->position);
        this->cam_rbt.applyRBTtoPosition(this->position);
        applyRotationToPosition(
            this->local_pose.roll,
            this->local_pose.pitch,
            this->local_pose.yaw,
            this->position
        );
    }
    else{
        this->position.detected = msg.tag_detected;
    }
}

void LandingTarget::cameraRBTWithIMUCallback(const atim::AtimPoseStamped &msg)
{
    bool tag_detected;

    if (msg.tag_detected) {
        this->position.x = msg.pose.position.x;
        this->position.y = msg.pose.position.y;
        this->position.z = msg.pose.position.z;
        this->position.detected = msg.tag_detected;


        this->cam_rbt.applyRBTtoPosition(this->position);
        applyRotationToPosition(
           msg.pose.orientation.x,
           msg.pose.orientation.y,
           msg.pose.orientation.z,
           msg.pose.orientation.w,
           this->position
        );
    }
    else{
        this->position.detected = msg.tag_detected;
    }
}

void LandingTarget::localPoseCallback(const geometry_msgs::PoseStamped &input)
{
    this->local_pose.x = input.pose.position.x;
    this->local_pose.y = input.pose.position.y;
    this->local_pose.z = input.pose.position.z;

    quat2euler(
        input.pose.orientation,
        &this->local_pose.roll,
        &this->local_pose.pitch,
        &this->local_pose.yaw
    );

    this->local_pose.roll = -1 * this->local_pose.roll;
    this->local_pose.pitch = -1 * this->local_pose.pitch;
    this->local_pose.yaw = 0 * this->local_pose.yaw;

    // ROS_INFO(
    //     "pose: %f\t%f\t%f",
    //     this->local_pose.roll,
    //     this->local_pose.pitch,
    //     this->local_pose.yaw
    // );
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
    correctedPose.pose.position.x = this->position.x;
    correctedPose.pose.position.y = this->position.y;
    correctedPose.pose.position.z = this->position.z;
    correctedPose.tag_detected = this->position.detected;

    correctedPose2.header.seq = seq;
    correctedPose2.header.stamp = time;
    correctedPose2.pose.position.x = this->position.x;
    correctedPose2.pose.position.y = this->position.y;
    correctedPose2.pose.position.z = this->position.z;

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

    double mount_roll = 0;
    double mount_pitch = M_PI;
    double mount_yaw = 0;
    // double mount_x = 0.067;
    // double mount_y = 0.0;
    // double mount_z = 0.07;

    double mount_x = 1.0;
    double mount_y = 0.0;
    double mount_z = 0.0;

    tf::TransformBroadcaster camera_imu_br;
    tf::Transform camera_imu_tf;
    tf::Quaternion q;

    q.setRPY(mount_roll, mount_pitch, mount_yaw);
    camera_imu_tf.setOrigin(tf::Vector3(mount_x, mount_y, mount_z));
    camera_imu_tf.setRotation(q);

    tf::TransformBroadcaster camera_image_br;
    tf::Transform camera_image_tf;
    tf::Quaternion q_image;
    q_image.setRPY(0, 0, 0);
    camera_image_tf.setOrigin(tf::Vector3(0.0, 0 , 1.0));
    camera_image_tf.setRotation(q_image);

    CameraMountRBT cam_rbt;
    LandingTarget *target;

    cam_rbt.initialize(
        mount_roll,
        mount_pitch,
        mount_yaw,
        mount_x,
        mount_y,
        mount_z
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
        camera_imu_br.sendTransform(
            tf::StampedTransform(
                camera_imu_tf, ros::Time::now(), "pixhawk_imu", "camera"
            )
        );

        camera_image_br.sendTransform(
            tf::StampedTransform(
                camera_image_tf, ros::Time::now(), "camera", "image"
            )
        );
        ros::spinOnce();
        rate.sleep();
        seq++;
    }

    return 0;
}
