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
	quad->subscribeToRadioIn();
    last_request = ros::Time::now();

    while (ros::ok()){
        // reset position controller errors
        if (quad->rc_in[6] > 1500) {
            // ROS_INFO("RC value is: %i ", quad->rc_in[6]);
            quad->position_controller->x.p_error = 0.0;
            quad->position_controller->x.i_error = 0.0;
            quad->position_controller->x.d_error = 0.0;

            quad->position_controller->y.p_error = 0.0;
            quad->position_controller->y.i_error = 0.0;
            quad->position_controller->y.d_error = 0.0;

            quad->position_controller->T.p_error = 0.0;
            quad->position_controller->T.i_error = 0.0;
            quad->position_controller->T.d_error = 0.0;
        }

        // publish quadrotor position controller
        pos.x = 0;
        pos.y = 0;
        pos.z = 1.5;
        quad->positionControllerCalculate(pos, last_request);
        quad->publishPositionControllerMessage(msg, seq, ros::Time::now());
        quad->publishPositionControllerStats(seq, ros::Time::now());
        // quad->printPositionController();

		// end
		seq++;
        last_request = ros::Time::now();
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
