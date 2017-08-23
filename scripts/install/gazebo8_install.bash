#!/bin/bash
set -e  # exit on first error
UBUNTU_NAME=`lsb_release -cs`
APT_SRC_URL=http://packages.osrfoundation.org/gazebo/ubuntu-stable
APT_SRC_LIST=/etc/apt/sources.list.d
GAZEBO_SRC_LIST=$APT_SRC_LIST/gazebo-stable.list
ROS_VERSION="kinetic"

install_gazebo8()
{
  # add gazebo apt sources list
  echo "deb $APT_SRC_URL $UBUNTU_NAME main" > $GAZEBO_SRC_LIST

  # get apt sources list key
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

  # install gazebo
  sudo apt-get update && apt-get install -qq -y ros-$ROS_VERSION-gazebo8-ros-pkgs
}


# RUN
ARCH_TYPE=`uname -m`
if [ ${ARCH_TYPE} == 'x86_64' ]; then
    install_gazebo8
else
    echo "Unsupported ARCH_TYPE [$ARCH_TYPE], NOT INSTALLING GAZEBO8!";
    exit -1;
fi
