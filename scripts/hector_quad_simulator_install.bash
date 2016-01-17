#!/bin/bash
HECTOR_PATH=$HOME
ROS_VERSION="indigo"

install_dependencies()
{
    apt-get install -y python-wstool \
        ros-indigo-cv-bridge \
        libeigen3-dev \
        ros-indigo-tf \
        ros-indigo-hardware_interface \
        build-essential \
        
}


install_hector_sim()
{
    source /opt/ros/$ROS_VERSION/setup.bash
    cd $HECTOR_PATH
    mkdir -p $HECTOR_PATH/hector_quadrotor_tutorial
    cd $HECTOR_PATH/hector_quadrotor_tutorial
    wstool init src https://raw.github.com/tu-darmstadt-ros-pkg/hector_quadrotor/$ROS_VERSION-devel/tutorials.rosinstall
    catkin_make -j2
    source devel/setup.bash
}


#RUN
install_dependencies
install_hector_sim
