#!/bin/bash
set -e  # exit on first error

ROS_VERSION="kinetic"
ROS_BASH="/opt/ros/$ROS_VERSION/setup.bash"
ROS_PACKAGES_URL='http://packages.ros.org/ros/ubuntu'
APT_KEYS_URL='http://packages.ros.org/ros.key'
APT_TARGETS="$(lsb_release -sc) main"
SOURCES_LIST_TARGET='/etc/apt/sources.list.d/ros-latest.list'

install() {
    # update sources.list and add apt-keys
    echo "deb $ROS_PACKAGES_URL $APT_TARGETS" > $SOURCES_LIST_TARGET
    wget $APT_KEYS_URL -O - | apt-key add -

    # update apt and install ros
    apt-get update -qq  # -qq makes apt-get less noisy
    apt-get install -y ros-$ROS_VERSION-desktop-full

    # initialize rosdep
    rosdep init
    rosdep update

    # env setup
    echo "source /opt/ros/$ROS_VERSION/setup.bash" >> $HOME/.bashrc

    # install ros
    apt-get install -y python-rosinstall python-catkin-tools

    # fix rosdep permissions
    rosdep fix-permissions
}


# RUN
install
