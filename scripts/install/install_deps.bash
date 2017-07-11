#!/bin/bash
set -e  # exit on first error
ROS_VERSION="kinetic"
CATKIN_PATH="$HOME/catkin_ws"
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# refresh permission
sudo -v

# dependencies available from apt
sudo apt-get install -qq -y \
    libcgal-* \
    libnlopt-* \
    protobuf-compiler \
    ros-$ROS_VERSION-mavros \
    ros-$ROS_VERSION-mavros-extras \
    ros-$ROS_VERSION-mavros-msgs

# dependencies from git
if [ ! -d "$CATKIN_PATH/src/Onboard-SDK-ROS" ]; then
    cd $CATKIN_PATH/src
    git clone https://github.com/dji-sdk/Onboard-SDK-ROS
    cd -
fi

# intall dependencies
sudo bash $SCRIPT_PATH/apriltags_michigan_install.bash
sudo bash $SCRIPT_PATH/apriltags_swathmore_install.bash
sudo bash $SCRIPT_PATH/apriltags_mit_install.bash
sudo bash $SCRIPT_PATH/ximea_install.bash
sudo bash $SCRIPT_PATH/pointgrey_install.bash
sudo bash $SCRIPT_PATH/djiosdk_install.bash

# create catkin workspace if it doesn't already exist
if [ ! -d $CATKIN_PATH/src ]; then
    mkdir -p $CATKIN_PATH/src
    catkin_init_workspace
fi

# link repo to catkin workspace
if [ ! -d "$CATKIN_PATH/src/atl" ]; then
     ln -sfn "$REPO_PATH" "$CATKIN_PATH/src/atl"
fi

# compile
cd "$CATKIN_PATH"
source /opt/ros/$ROS_VERSION/setup.bash
catkin build
