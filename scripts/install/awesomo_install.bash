#!/bin/bash
set -e  # exit on first error
CATKIN_PATH="$HOME/catkin_ws"
SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_PATH=$(dirname "$SCRIPT_PATH")

# refresh permission
sudo -v

# link repo to catkin workspace
ln -sfn "$REPO_PATH" "$CATKIN_PATH/src/awesomo"

# compile
cd "$CATKIN_PATH"
source /opt/ros/indigo/setup.bash
catkin build
