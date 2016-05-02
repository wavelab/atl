#include "awesomo/ros/quadrotor_node.hpp"


int main(int argc, char **argv)
{
    // setup
    ros::init(argc, argv, "awesomo");
    ros::Time now;
    ros::Time last_request;
    ros::Rate rate(50.0);  // publishing rate MUST be faster than 2Hz
    Quadrotor quad;

    ROS_INFO("running ...");
	int seq = 1;
	int index = 0;

    while (ros::ok()){
        // quad.runMission(position);
        last_request = ros::Time::now();
        now = last_request;

		// publish


		// end
		seq++;
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
