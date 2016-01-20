#!/bin/bash
ROS_VERSION="indigo"
HUSKY_PATH=$HOME

install_husky_sim()
{
    apt-get install -y ros-indigo-husky-simulator
}

#run
install_husky_sim
