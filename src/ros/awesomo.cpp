#include "awesomo/ros/quadrotor.hpp"


int main(int argc, char **argv)
{
    // setup
    ros::init(argc, argv, "awesomo");
    ros::NodeHandle node_handle;
    ros::Rate rate(50.0);
    ros::Time last_request;
    geometry_msgs::PoseStamped msg;

	int seq = 1;
	int index = 0;
	int timeout = 0;
    Quadrotor *quad;
    Position pos;
    std::string position_controller_config;
    std::map<std::string, std::string> configs;

    // get configuration paths
	node_handle.getParam("/position_controller", position_controller_config);
	configs["position_controller"] = position_controller_config;

	// setup quad
    ROS_INFO("running ...");
    quad = new Quadrotor(configs);
	quad->subscribeToPose();
    last_request = ros::Time::now();

    while (ros::ok()){
        pos.x = 0;
        pos.y = 0;
        pos.z = 1.5;
        quad->positionControllerCalculate(pos, last_request);
        quad->publishPositionControllerMessage(msg, seq, ros::Time::now());
        quad->printPositionController();
        last_request = ros::Time::now();

		// end
		seq++;
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
