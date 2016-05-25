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
    Quadrotor *quad;
    std::string position_controller_config;
    std::string carrot_controller_config;
    std::map<std::string, std::string> configs;

    // get configuration paths
	node_handle.getParam("/position_controller", position_controller_config);
	node_handle.getParam("/carrot_controller", carrot_controller_config);
	configs["position_controller"] = position_controller_config;
	configs["carrot_controller"] = carrot_controller_config;

	// setup quad
    ROS_INFO("running ...");
    quad = new Quadrotor(configs);
    last_request = ros::Time::now();

    while (ros::ok()){
        // quad run mission
        quad->runMission(msg, seq, last_request);

		// end
		seq++;
        last_request = ros::Time::now();
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
