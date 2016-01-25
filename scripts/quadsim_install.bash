#!/bin/bash


install_dependencies()
{
    # base
    apt-get install -y \
        gawk \
        make \
        git \
        curl

    # mavlink
    apt-get install -y \
        g++ \
        python-pip \
        python-matplotlib \
        python-serial \
        python-wxgtk2.8 \
        python-scipy \
        python-opencv \
        python-numpy \
        python-pyparsing \
        ccache \
        realpath

}

install_mavproxy()
{
    sudo pip2 install pymavlink \
        MAVProxy \
        catkin_pkg \
        -- upgrade
}

install_ardupilot()
{
    mkdir -p ~/ardupilot_simulation
    cd ~/arduopilot_simulation
    git clone https://github.com/erlerobot/ardupilot
    cd ardupilot
    git checkout gazebo
}

install_jsbsim()
{
    cd ~/ardupilot_simulation
    git clone git://github.com/tridge/jsbsim.git
    # Additional dependencies required
    sudo apt-get install libtool automake autoconf libexpat1-dev -y
    cd jsbsim
    ./autogen.sh --enable-libraries
    make -j2
    sudo make install
}

install_rosdeps()
{
    apt-get install -y \
        ros-indigo-octomap-msgs \
        ros-indigo-joy \
        ros-indigo-geodesy \
        ros-indigo-octomap-ros \
        unzip
}

install_drcsim()
{
    # add OSRF repo and install drcsim?
    sh -c 'echo "deb http://packages.osrfoundation.org/drc/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/drc-latest.list'
    wget http://packages.osrfoundation.org/drc.key -O - |  apt-key add -
    apt-get update
    apt-get -y install drcsim

}

ros_setup()
{
    mkdir -p ~/ardupilot_simulation/ros_catkin_ws/src
    cd ~/ardupilot_simulation/ros_catkin_ws/src
    catkin_init_workspace
    cd ..
    catkin_make
    source /devel/setup.bash

    cd src/
    git clone https://github.com/erlerobot/ardupilot_sitl_gazebo_plugin
    git clone https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/

    # Rotors simulation
    git clone https://github.com/erlerobot/rotors_simulator -b sonar_plugin

    ## mav comm
    git clone https://github.com/PX4/mav_comm.git

    ## glog catkin
    git clone https://github.com/ethz-asl/glog_catkin.git

    ## catkin simple
    git clone https://github.com/catkin/catkin_simple.git

    # installation of `mavros` from its source code:
    cd ~/ardupilot_simulation/ros_catkin_ws
    wstool init src     # (if not already initialized)
    wstool set -t src mavros --git https://github.com/erlerobot/mavros.git
    wstool update -t src

    ## compile.
    cd ~/ardupilot_simulation/ros_catkin_ws
    catkin_make
}

install_gazebo_models()
{
    git clone https://github.com/erlerobot/erle_gazebo_models
    mv erle_gazebo_models ~/.gazebo/models
}


# RUN
install_dependencies
install_mavproxy
install_ardupilot
install_jsbsim
install_rosdeps
install_drcsim
ros_setup
install_gazebo_models
