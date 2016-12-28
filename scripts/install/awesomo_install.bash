#!/bin/bash
set -e  # exit on first error
CATKIN_PATH="$HOME/catkin_ws"
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_PATH=$(dirname "$SCRIPT_PATH")

# refresh permission
sudo -v

# intall dependencies
sudo bash ximea_install.bash
sudo bash pointgrey_install.bash
sudo bash swathmore_apriltags.bash

# link repo to catkin workspace
ln -sfn "$REPO_PATH" "$CATKIN_PATH/src/awesomo"

# compile
cd "$CATKIN_PATH"
source /opt/ros/indigo/setup.bash
catkin build
