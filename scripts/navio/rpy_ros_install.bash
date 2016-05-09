#!/bin/bash
set -e  # exit on first error

SOURCES_LIST=/etc/apt/sources.list

RASPBIAN_VERSION=jessie
RASPBIAN_PKG_URL=http://mirrordirector.raspbian.org/raspbian/

ROS_SOURCES_LIST=/etc/apt/sources.list.d/ros-latest.list
ROS_VERSION=indigo
ROS_KEY_URL=https://raw.githubusercontent.com/ros/rosdistro/master/ros.key
ROS_PKG_URL=http://packages.ros.org/ros/ubuntu


setup_ros_repos()
{
    echo "deb $ROS_PKG_URL $RASPBIAN_VERSION main" > $ROS_SOURCES_LIST
    wget $ROS_KEY_URL -O - | sudo apt-key add -

    sudo apt-get update
    sudo apt-get upgrade -y
}

bootstrap_dependencies()
{
    sudo apt-get install -y \
        python-pip \
        python-setuptools \
        python-yaml \
        python-distribute \
        python-docutils \
        python-dateutil \
        python-six \
        python-empy \
        python-nose \
        libboost-all-dev \
        libbz2-dev

    sudo pip install \
        rosdep \
        rosinstall \
        rosinstall_generator \
        wstool
}

init_rosdep()
{
    sudo rosdep init
    rosdep update  # IMPORTANT! DO NOT RUN AS ROOT!
}

install_ros()
{
    mkdir $HOME/ros_catkin_ws
    cd $HOME/ros_catkin_ws

    rosinstall_generator desktop \
        --rosdistro $ROS_VERSION \
        --deps \
        --wet-only \
        --exclude roslisp \
        --tar > $ROS_VERSION-desktop-wet.rosinstall

    wstool init src $ROS_VERSION-desktop-wet.rosinstall

    cd $HOME
}

resolve_dependencies()
{
    # Before you can build your catkin workspace, you need to make sure that
    # you have all the required dependencies. We use the rosdep tool for this,
    # however, a couple of dependencies are not available in the repositories.
    # They must be manually built first.

    # setup sources.list
    sudo apt-get install -y checkinstall cmake
    sudo echo "deb-src $RASPBIAN_PKG_URL testing main contrib non-free rpi" >> $SOURCES_LIST
    sudo apt-get update -y

    # navigate to external source
    mkdir $HOME/ros_catkin_ws/external_src
    cd $HOME/ros_catkin_ws/external_src

    # libconsole-bridge-dev
    sudo apt-get -y build-dep console-bridge
    apt-get source -b console-bridge
    sudo dpkg -i \
        libconsole-bridge0.2*.deb \
        libconsole-bridge-dev_*.deb

    # liblz4-dev
    apt-get source -b lz4
    sudo dpkg -i liblz4-*.deb

    # liburdfdom-headers-dev
    git clone https://github.com/ros/urdfdom_headers.git
    cd urdfdom_headers
    cmake .
    sudo checkinstall make install
    cd ..

    # liburdfdom-dev
    sudo apt-get install libboost-test-dev libtinyxml-dev
    git clone https://github.com/ros/urdfdom.git
    cd urdfdom
    cmake .
    sudo checkinstall make install
    cd ..

    # collada-dom-dev
    sudo apt-get install libboost-filesystem-dev libxml2-dev
    wget http://downloads.sourceforge.net/project/collada-dom/Collada%20DOM/Collada%20DOM%202.4/collada-dom-2.4.0.tgz
    tar -xzf collada-dom-2.4.0.tgz
    cd collada-dom-2.4.0
    cmake .
    sudo checkinstall make install
    cd ..

    # resolve remaining dependencies
    cd $HOME/ros_catkin_ws
    rosdep install \
        --from-paths src \
        --ignore-src \
        --rosdistro $ROS_VERSION -y -r \
        --os=debian:$RASPBIAN_VERSION
}

build_catkin_ws()
{
    cd /root/ros_catkin_ws

    sudo mkdir -p /opt/ros/$ROS_VERSION
    sudo ./src/catkin/bin/catkin_make_isolated \
        --install \
        -DCMAKE_BUILD_TYPE=Release \
        --install-space /opt/ros/$ROS_VERSION -j1 

    cd $HOME
}



# MAIN
setup_ros_repos
bootstrap_dependencies
init_rosdep
install_ros
resolve_dependencies
build_catkin_ws
