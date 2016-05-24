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
    Eigen::Vector3d position;
    Eigen::Vector3d carrot;
    Eigen::Vector3d wp;
    std::string position_controller_config;
    std::string carrot_controller_config;
    std::map<std::string, std::string> configs;

    // get configuration paths
	node_handle.getParam("/position_controller", position_controller_config);
	configs["position_controller"] = position_controller_config;
	node_handle.getParam("/carrot_controller", carrot_controller_config);
	configs["carrot_controller"] = carrot_controller_config;

	// setup quad
    ROS_INFO("running ...");
    quad = new Quadrotor(configs);
	quad->subscribeToPose();
	quad->subscribeToRadioIn();
    last_request = ros::Time::now();
    int initlialize_carrot = 0;

    while (ros::ok()){
        // reset position controller errors
        if (quad->rc_in[6] < 1500) {
            quad->position_controller->x.sum_error = 0.0;
            quad->position_controller->x.prev_error = 0.0;
            quad->position_controller->x.output = 0.0;

            quad->position_controller->y.sum_error = 0.0;
            quad->position_controller->y.prev_error = 0.0;
            quad->position_controller->y.output = 0.0;

            quad->position_controller->T.sum_error = 0.0;
            quad->position_controller->T.prev_error = 0.0;
            quad->position_controller->T.output = 0.0;

            // configure setpoint to be where the quad currently is
            pos.x = quad->pose.x;
            pos.y = quad->pose.y;
            pos.z = quad->pose.z + 3;

            wp << pos.x, pos.y, pos.z;
            quad->carrot_controller->waypoints.push_back(wp);

            wp << pos.x + 5, pos.y, pos.z;
            quad->carrot_controller->waypoints.push_back(wp);

            wp << pos.x + 5, pos.y + 5, pos.z;
            quad->carrot_controller->waypoints.push_back(wp);

            wp << pos.x, pos.y + 5, pos.z;
            quad->carrot_controller->waypoints.push_back(wp);

            wp << pos.x + 5, pos.y, pos.z;
            quad->carrot_controller->waypoints.push_back(wp);

            wp << pos.x, pos.y, pos.z;
            quad->carrot_controller->waypoints.push_back(wp);
            quad->carrot_controller->initialized;

        } else {
            // carrot controller
            position << quad->pose.x, quad->pose.y, quad->pose.z;
            quad->carrot_controller->update(position, carrot);
            pos.x = carrot(0);
            pos.y = carrot(1);
            pos.z = carrot(2);

        }

        // publish quadrotor position controller
        quad->positionControllerCalculate(pos, last_request);
        quad->publishPositionControllerMessage(msg, seq, ros::Time::now());
        quad->publishPositionControllerStats(seq, ros::Time::now());

		// end
		seq++;
        last_request = ros::Time::now();
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
