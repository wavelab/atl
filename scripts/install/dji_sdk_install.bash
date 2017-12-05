#!/bin/bash
set -e  # exit on first error
DOWNLOAD_PATH="/usr/local/src"
CATKIN_PATH="$HOME/catkin_ws"
CORE_SDK_REPO_URL="https://github.com/dji-sdk/Onboard-SDK"
CORE_SDK_VER="3.4"
ROS_SDK_REPO_URL="https://github.com/dji-sdk/Onboard-SDK-ROS"
ROS_SDK_VER="3.3"

install_core_sdk()
{
    # download DJI Onboard Core SDK
    cd $DOWNLOAD_PATH
    if [ ! -d $DOWNLOAD_PATH/Onboard-SDK ]; then
        sudo git clone $CORE_SDK_REPO_URL
        sudo git checkout $CORE_SDK_VER
    fi

    # compile and install
    cd Onboard-SDK
    sudo mkdir -p build
    cd build
    sudo cmake ..
    sudo make
    sudo make install djiosdk-core
}

install_ros_sdk()
{
    # download DJI Onboard ROS SDK
    mkdir -p $CATKIN_PATH/src
    cd $CATKIN_PATH/src
    if [ ! -d $CATKIN_PATH/src/Onboard-SDK-ROS ]; then
        git clone $ROS_SDK_REPO_URL
        sudo git checkout $ROS_SDK_VER
    fi
}


# RUN
install_core_sdk
install_ros_sdk
