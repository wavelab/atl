#!/bin/bash
AWESOMO_BUILD_PATH=$HOME/awesomo_build/
AWESOMO_CATKIN_PATH=$HOME/awesomo_catkin/


install_dependencies()
{
    # create catkin workspace
    mkdir -p $AWESOMO_CATKIN_PATH/src
    cd $AWESOMO_CATKIN_PATH/src
    catkin_init_workspace
    source devel/setup.bash

    # install required ros packages
    sudo apt-get install ros-indigo-octomap-msgs
}

install_ardupilot()
{
    # clone ardupilot
    cd $AWESOMO_BUILD_PATH
    mkdir -p $AWESOMO_BUILD_PATH
    git clone https://github.com/alexbuyval/ardupilot
    git checkout RangeFinderSITL2

    # clone all necessary ROS packages
    roscd
    cd $AWESOMO_CATKIN_PATH/src
    git clone https://alexbuyval@bitbucket.org/alexbuyval/arducopter_sitl_ros.git
    git clone https://github.com/PX4/mav_comm.git
    git clone https://github.com/alexbuyval/rotors_simulator.git
    git clone https://github.com/ethz-asl/glog_catkin.git
    git clone https://github.com/catkin/catkin_simple.git
    cd rotors_simulator
    git checkout sonar_plugin
    cd ../..
    wstool init src
    wstool set -t src mavros --git https://github.com/alexbuyval/mavros.git
    wstool update -t src
    rosdep install --from-paths src --ignore-src --rosdistro indigo -y

    # build awesomo catkin path
    cd $AWESOMO_CATKIN_PATH
    catkin_make  # compile the files
    export PATH=$PATH:$AWESOMO_BUILD_PATH/ardupilot/Tools/autotest
}

start_sim()
{
    cd $AWESOMO_BUILD_PATH/ardupilot/ArduCopter
    sim_vehicle.sh -f arducopter_sitl_ros --console
}
