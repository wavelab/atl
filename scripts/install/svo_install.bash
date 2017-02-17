#!/bin/sh
set -e
ROS_DISTRO="indigo"
# ROS_DISTRO="kinetic"
CATKIN_WS=$HOME/catkin_ws/

install_sophus()
{
    cd /tmp
    git clone https://github.com/strasdat/Sophus.git
    cd Sophus
    git checkout a621ff
    mkdir build
    cd build
    cmake ..
    make
}

install_fast()
{
    cd /tmp
    git clone https://github.com/uzh-rpg/fast.git
    cd fast
    mkdir build
    cd build
    cmake ..
    make
}

install_g2o()
{
    cd /tmp
    git clone https://github.com/RainerKuemmerle/g2o.git
    cd g2o
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
}

install_vikit()
{
    cd $CATKIN_WS/src
    git clone https://github.com/uzh-rpg/rpg_vikit.git
}

install_svo()
{
    sudo apt-get install ros-$ROS_DISTRO-cmake-modules
    cd $CATKIN_WS/src
    git clone https://github.com/uzh-rpg/rpg_svo.git
}


# RUN
install_sophus
install_fast
install_g2o
install_vikit
install_svo
