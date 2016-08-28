#include <iostream>

#include <ros/ros.h>

#include "awesomo/gimbal.hpp"


int main(int argc, char **argv)
{
    int retval;
    Gimbal *gimbal;
    std::string gimbal_config;
    std::map<std::string, std::string> configs;

    // setup
    ros::init(argc, argv, "awesomo_gimbal");
    ros::NodeHandle node_handle;
    ros::Rate rate(10.0);
    ros::Time last_request;

    // setup awesomo_gimbal
    node_handle.getParam("/gimbal", gimbal_config);
    configs["gimbal"] = gimbal_config;
    gimbal = new Gimbal(configs);
    last_request = ros::Time::now();

    // gimbal->setGimbalAngles(0, -20, 0);
    SBGC *sbgc = new SBGC("/dev/ttyUSB0");
    sbgc->connect();
    sbgc->on();

    // loop
    while (ros::ok()){
        // gimbal->setGimbalAngles(0, -30, 0);
        retval = sbgc->getRealtimeData();
        if (retval == 0) {
            sbgc->data.printData();
        }

        // end
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
