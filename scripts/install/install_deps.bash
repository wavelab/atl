#!/bin/bash
set -e  # exit on first error
ROS_VERSION="kinetic"
CATKIN_PATH="$HOME/catkin_ws"
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# refresh permission
sudo -v

# source ros
source /opt/ros/$ROS_VERSION/setup.bash

# dependencies available from apt
sudo apt-get install -qq -y \
    libcgal-* \
    libnlopt-* \
    protobuf-compiler \
    python-catkin-tools \
    ros-$ROS_VERSION-mavros \
    ros-$ROS_VERSION-mavros-extras \
    ros-$ROS_VERSION-mavros-msgs


# setup catkin workspace
if [ ! -d $CATKIN_PATH ]; then
    mkdir -p $CATKIN_PATH
    cd $CATKIN_PATH
    catkin init
    cd -
fi

# intall dependencies
sudo bash $SCRIPT_PATH/apriltags_michigan_install.bash
sudo bash $SCRIPT_PATH/apriltags_swathmore_install.bash
sudo bash $SCRIPT_PATH/apriltags_mit_install.bash
# sudo bash $SCRIPT_PATH/ximea_install.bash
# sudo bash $SCRIPT_PATH/pointgrey_install.bash
# sudo bash $SCRIPT_PATH/dji_sdk_install.bash
#
# # link repo to catkin workspace
# if [ ! -d $CATKIN_PATH/src/atl ]; then
#      ln -sfn $REPO_PATH $CATKIN_PATH/src/atl
# fi
#
# # compile
# cd $CATKIN_PATH
# source /opt/ros/$ROS_VERSION/setup.bash
# catkin build
