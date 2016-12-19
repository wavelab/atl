#!/bin/bash
set -e  # exit on first error

ROS_VERSION="indigo"
ROS_BASH="opt/ros/$ROS_VERSION/setup.bash"
ROS_PACKAGES_URL="http://packages.ros.org/ros/ubuntu"
APT_KEYS_URL="http://packages.ros.org/ros.key"
APT_TARGETS="$(lsb_release -sc) main universe restricted multiverse"
SOURCES_LIST_TARGET="/etc/apt/sources.list.d/ros-latest.list"


add_repositories()
{
    # add the main, universe and multiverse repo
    sudo add-apt-repository universe
    sudo add-apt-repository multiverse
    sudo apt-get update

    # install git
    sudo apt-get install git
}

install_ros()
{
    # update sources.list and add apt-get keys
    # echo "deb $ROS_PACKAGES_URL $APT_TARGETS" > $SOURCES_LIST_TARGET
    # wget $APT_KEYS_URL -O - | sudo apt-key add -

    # update and install ros base
    # apt-get update -qq
    apt-get install ros-indigo-desktop

    # initialize rosdep
    rosdep init
    rosdep update

    # env setup
    echo "source /opt/ros/$ROS_VERSION/setup.bash" >> $HOME/.bashrc

    # install ros
    apt-get install -y python-rosinstall

    # fix rosdep permissions
    rosdep fix-permissions
}

setup_catkin()
{
    cd $HOME
    source .bashrc
    mkdir -p $HOME/catkin_ws/src
    cd catkin_ws/src
    catkin_init_workspace

    cd $HOME/catkin_ws/
    catkin_make
    echo "source $HOME/catkin_ws/devel/setup.bash" >> $HOME/.bashrc
}

install_atim()
{
    cd $HOME/catkin_ws/src
    git clone https://github.com/wavelab/atim.git

    # install atim dependensies
    cd atim
    sudo bash scripts/install_scripts/pointgrey_arm_install.bash
    sudo bash scripts/install_scripts/ximea_install.bash
    sudo bash scripts/install_scripts/swathmore_apriltags.bash

    cd $HOME/catkin_ws
    catkin_make install
}

install_awesomo()
{
    cd $HOME/catkin_ws/src

    # install awesomo dependensies
    sudo apt-get install ros-indio-mavros-*

    # install awesomo
    git clone https://github.com/wavelab/awesomo.git
    cd $HOME/catkin_ws
    catkin_make install
}

setup_wifi()
{
    # connect using the command line interface of network manager
    nmcli d wifi connect AwesomoTower
}

setup_permissions()
{
    sudo usermod -a -G px4 ubuntu
    sudo usermod -a -G tty ubuntu
}


# main
add_repositories
install_ros
setup_catkin
install_atim
install_awesomo
setup_wifi
