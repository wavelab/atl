#!/bin/sh
set -e
ROS_VERSION="kinetic"
CATKIN_WS=$HOME/catkin_ws/

check_user()
{
    if [[ $EUID -eq 0 ]]; then
        echo "Do not run this script as root!"
        exit 1
    fi
}

install_sophus()
{
    cd /usr/local/src/
    if [ ! -d Sophus ]; then
    	git clone https://github.com/strasdat/Sophus.git
    fi
    cd Sophus
    git checkout a621ff
    mkdir -p build
    cd build
    cmake ..
    make
}

install_fast()
{
    cd /usr/local/src/
    if [ ! -d fast ]; then
    	git clone https://github.com/uzh-rpg/fast.git
    fi
    cd fast
    mkdir -p build
    cd build
    cmake ..
    make
}

install_g2o()
{
    cd /usr/local/src/
    if [ ! -d g2o ]; then
        git clone https://github.com/RainerKuemmerle/g2o.git
    fi
    cd g2o
    mkdir -p build
    cd build
    cmake ..
    make
    sudo make install
}

install_vikit()
{
    cd $CATKIN_WS/src
    if [ ! -d rpg_vikit ]; then
        git clone https://github.com/uzh-rpg/rpg_vikit.git
    fi
}

install_dependencies()
{
    sudo apt-get install -y \
        git \
        cmake \
        libeigen3-dev

    sudo bash -c "$(declare -f install_sophus); install_sophus"
    sudo bash -c "$(declare -f install_fast); install_fast"
    sudo bash -c "$(declare -f install_g2o); g2o"
	install_vikit
}

install_svo()
{
    sudo apt-get install ros-$ROS_VERSION-cmake-modules
    cd $CATKIN_WS/src
    if [ ! -d rpg_svo ]; then
        git clone https://github.com/uzh-rpg/rpg_svo.git
    fi

    # build svo
    cd $CATKIN_WS
    source /opt/ros/$ROS_VERSION/setup.bash
    catkin build
}

# RUN
check_user
install_dependencies
install_svo
