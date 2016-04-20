#include "awesomo/ros/camera_node.hpp"

#define MOCAP_TOPIC "/awesomo/mocap/pose"
#include <std_msgs/Float64.h>


float pos_x;
float pos_y;
float pos_z;
float quat_x;
float quat_y;
float quat_z;
float quat_w;

static double tic(void)
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((double) t.tv_sec + ((double) t.tv_usec) / 1000000.0);
}

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
    // geometry_msgs::PoseWithCovariance pose_msg;
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    std_msgs::Float64 stupid;

    ros::init(argc, argv, "awesomo_camera");
    ros::NodeHandle n;
    ros::Rate rate(100);
    ros::Subscriber mocap_subscriber;
    ros::Publisher publisher;
    ros::Publisher publisher2;
    ros::Publisher publisher3;

    // setup
    seq = 0;
    timeout = 0;
    rotation_matrix(M_PI_2, -M_PI_2, 0.0, rot_mat);

    // ROS specifics
    // publisher = n.advertise<geometry_msgs::PoseStamped>(ROS_TOPIC, 100);
    publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(ROS_TOPIC, 100);
    publisher2 = n.advertise<geometry_msgs::PoseStamped>(ROS_TOPIC2, 100);
    publisher3 = n.advertise<std_msgs::Float64>("/awesomo/hertz", 100);

    // camera specifics
    Camera cam(0, CAMERA_FIREFLY);
    cam.loadConfig("default", FIREFLY_640);
    cam.loadConfig("320", FIREFLY_320);
    cam.loadConfig("160", FIREFLY_160);
    cam.initCamera("320");
    ROS_INFO("Camera node is publishing pose data!");


    // junk code
    int key_input;
    int frame_index;
    double last_tic;
    geometry_msgs::PoseStamped msg;
    mocap_subscriber = n.subscribe(MOCAP_TOPIC, 100, mocapCallback);
    frame_index = 0;
    last_tic = tic();

    // ROS node loop
    while (ros::ok()) {
        pose_estimates = cam.step(timeout);

        // key_input = cvWaitKey(10);

        frame_index++;
        if (frame_index % 10 == 0) {

            // publish poses
            for (int i = 0; i < pose_estimates.size(); i++) {
                // build pose message
                pose = pose_estimates[i];
                build_pose_stamped_cov_msg(seq, pose, rot_mat, pose_msg);
                publisher.publish(pose_msg);


                msg.header.stamp = pose_msg.header.stamp;
                msg.header.seq = seq;
                msg.header.frame_id = "awesomo_position";
                msg.pose.position.x = pos_x;
                msg.pose.position.y = pos_y;
                msg.pose.position.z = pos_z;
                msg.pose.orientation.x = quat_x;
                msg.pose.orientation.y = quat_y;
                msg.pose.orientation.z = quat_z;
                msg.pose.orientation.w = quat_w;
                publisher2.publish(msg);

                double t = tic();
                float fps = 10.0 / (t - last_tic);
                cout << "\t" << 10.0 / (t - last_tic) << " fps" << endl;
                last_tic = t;
                stupid.data = fps;
                publisher3.publish(stupid);

                seq++;
            }

        }

        // send last known estimate if tag not detected
        // if (pose_estimates.size() == 0) {
        //     // publish and spin
        //     pose_msg.header.seq = seq;
        //     pose_msg.header.stamp = ros::Time::now();
        //     // publisher.publish(pose_msg);
        //
        //     // record
        //     // if ((char) key_input == 49) {
        //         publisher.publish(pose_msg);
        //
        //         msg.header.stamp = pose_msg.header.stamp;
        //         msg.header.seq = seq;
        //         msg.header.frame_id = "awesomo_position";
        //         msg.pose.position.x = pos_x;
        //         msg.pose.position.y = pos_y;
        //         msg.pose.position.z = pos_z;
        //         msg.pose.orientation.x = quat_x;
        //         msg.pose.orientation.y = quat_y;
        //         msg.pose.orientation.z = quat_z;
        //         msg.pose.orientation.w = quat_w;
        //
        //         publisher2.publish(msg);
        //     // }
        //
        //     // update
        //     seq++;
        // }

        // sleep
        rate.sleep();
        ros::spinOnce();
    }
    ROS_INFO("Camera node exited!");

    return 0;
}
