#!/bin/bash
set -e
ROS_DISTRO="indigo"
CATKIN_DIR="$HOME/catkin_ws"


install_catkin_tools()
{
    # Check if already installed
    if type catkin > /dev/null 2>&1; then
        echo "Catkin tools is already installed"
    else
        echo "Installing catkin tools ..."
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
        wget -qO - http://packages.ros.org/ros.key | sudo apt-key add -
        sudo apt-get -qq update
        sudo apt-get -qq install python-catkin-tools > /dev/null
        echo "Catkin tools installed successfully."
    fi
}

create_catkin_ws()
{
    # Check if workspace exists
    if [ -e "$CATKIN_DIR/.catkin_workspace" ] || [ -d "$CATKIN_DIR/.catkin_tools" ]; then
        echo "Catkin workspace detected at ~/catkin_ws"
        rm -rf build devel install logs
    else
        echo "Creating catkin workspace in $HOME/catkin_ws ..."
        source /opt/ros/$ROS_DISTRO/setup.bash
        mkdir -p "$HOME/catkin_ws/src"
        cd "$HOME/catkin_ws/src"
        catkin_init_workspace
        cd "$HOME/catkin_ws"
        catkin_make
        echo "Catkin workspace created successfully."
    fi
}


# RUN
# install_catkin_tools
create_catkin_ws
