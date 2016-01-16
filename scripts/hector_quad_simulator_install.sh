#!/bin/sh
HECTOR_PATH=$HOME


install_hector_sim()
{
    cd $HECTOR_PATH
    mkdir $HECTOR_PATH/hector_quadrotor_tutorial
    cd $HECTOR_PATH/hector_quadrotor_tutorial
    wstool init src https://raw.github.com/tu-darmstadt-ros-pkg/hector_quadrotor/hydro-devel/tutorials.rosinstall
    catkin_make -j2
    source devel/setup.bash
}


#RUN
install_hector_sim
