#include "core.hpp"





mavros_msgs::State current_state;

static void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

int core(int argc, char **argv)
{
    ros::Time last_request;
    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::CommandBool arm_cmd;

    // setup
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    // local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    // arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    // set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // geometry_msgs::PoseStamped pose;
    // pose.pose.position.x = 0;
    // pose.pose.position.y = 0;
    // pose.pose.position.z = 2;

    // // send a few setpoints before starting
    // for (int i = 100; ros::ok() && i > 0; --i) {
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    //
    // offb_set_mode.request.custom_mode = "OFFBOARD";
    // arm_cmd.request.value = true;
    // last_request = ros::Time::now();
    //
    // while (ros::ok()) {
    //     if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
    //         if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.success) {
    //             ROS_INFO("Offboard enabled");
    //         }
    //         last_request = ros::Time::now();
    //     } else {
    //         if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
    //             if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
    //                 ROS_INFO("Vehicle armed");
    //             }
    //             last_request = ros::Time::now();
    //         }
    //     }
    //     local_pos_pub.publish(pose);
    //
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    return 0;
}
