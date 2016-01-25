#!/bin/bash
ROS_VERSION="indigo"
HUSKY_PATH=$HOME


install_husky_sim()
{
    apt-get install -y ros-$ROS_VERSION-husky-simulator
}


# RUN
install_husky_sim
