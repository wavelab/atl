#!/bin/sh

install_dependencies()
{
    apt-get install -y \
        ros-indigo-desktop-full
}
install_hector_sim()
{
    cd $HOME
    mkdir ~/hector_quadrotor_tutorial
    cd ~/hector_quadrotor_tutorial
    wstool init src https://raw.github.com/tu-darmstadt-ros-pkg/hector_quadrotor/hydro-devel/tutorials.rosinstall
    catkin_make -j2
    source devel/setup.bash
}

#RUN
install_dependencies
install_hector_sim

cd $HOME
