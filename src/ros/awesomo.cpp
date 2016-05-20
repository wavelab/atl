#include "awesomo/ros/quadrotor.hpp"


int main(int argc, char **argv)
{
    // setup
    ros::init(argc, argv, "awesomo");
    ros::NodeHandle node_handle;
    ros::Rate rate(50.0);  // publishing rate MUST be faster than 2Hz
    ros::Time last_request;
    geometry_msgs::PoseStamped msg;

	int seq = 1;
	int index = 0;
	int timeout = 0;
    Quadrotor quad;
    Position pos;
    std::string camera_config_path;
    std::string position_controller_config_path;

    ROS_INFO("running ...");

    // get configuration paths
	node_handle.getParam(
	    "/camera_config_path",
	    camera_config_path
	);
	node_handle.getParam(
	    "/position_controller_config_path",
	    position_controller_config_path
	);

	// setup quad
	quad.subscribeToPose();
    last_request = ros::Time::now();

    while (ros::ok()){
        pos.x = 0;
        pos.y = 0;
        pos.z = 0;
        quad.positionControllerCalculate(pos, last_request);
        quad.publishPositionControllerMessage(msg, seq, ros::Time::now());
        last_request = ros::Time::now();

		// end
		seq++;
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
