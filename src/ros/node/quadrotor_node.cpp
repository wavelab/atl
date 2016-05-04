#include "awesomo/ros/node/quadrotor_node.hpp"


int main(int argc, char **argv)
{
    // setup
    ros::init(argc, argv, "awesomo");
    ros::NodeHandle node_handle;
    ros::Rate rate(50.0);  // publishing rate MUST be faster than 2Hz
    ros::Time now;
    ros::Time last_request;

	int seq = 1;
	int index = 0;
	int timeout = 0;
    Quadrotor quad;
    std::string camera_config_path;
    std::string position_controller_config_path;

    ROS_INFO("running ...");

    // setup quadrotor
	node_handle.getParam(
	    "/camera_config_path",
	    camera_config_path
	);
	node_handle.getParam(
	    "/position_controller_config_path",
	    position_controller_config_path
	);
    // quad.cam = new Camera(camera_config_path);

    while (ros::ok()){
        // quad.runMission(position);
        last_request = ros::Time::now();
        now = last_request;

		// publish
        // quad.cam->step(timeout);

		// end
		seq++;
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
