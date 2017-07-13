#!/bin/bash
set -e  # exit on first error
UBUNTU_NAME=`lsb_release -cs`
APT_SRC_URL=http://packages.osrfoundation.org/gazebo/ubuntu-stable
APT_SRC_LIST=/etc/apt/sources.list.d
GAZEBO_SRC_LIST=$APT_SRC_LIST/gazebo-stable.list

install_gazebo8()
{
  # add gazebo apt sources list
  echo "deb $APT_SRC_URL $UBUNTU_NAME main" > $GAZEBO_SRC_LIST

  # get apt sources list key
  wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

  # install gazebo
  sudo apt-get update
  sudo apt-get install gazebo8
}


# RUN
install_gazebo8
