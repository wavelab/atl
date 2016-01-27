#!/bin/bash
set -e  # halt on the first mistake
PX4_FIRMWARE_INSTALL_LOCATION="$HOME/px4_firmware"
ROS_VERSION="indigo"


# instructions are taken from here:
# http://dev.px4.io/starting-installing-linux.html


# apparently this is required followed by a logout?
sudo usermod -a -G dialout $USER

install_p4x_deps()
{
    sudo add-apt-repository ppa:george-edison55/cmake-3.x -y
    sudo apt-get update 
    sudo apt-get install python-argparse \
        git-core \
        wget zip \
        python-empy \
        qtcreator \
        cmake \
        build-essential \
        genromfs -y

    # simulation tools
    sudo apt-get install ant \
                        protobuf-compiler\
                        libeigen3-dev \
                        libopencv-dev\
                        openjdk-7-jdk \
                        openjdk-7-jre \
                        clang-3.5 \
                        lldb-3.5 -y

}

px4_install_stuff()
{
    # remove serial modem manager, website says its ok...
    # here it goes then...a
    # acctaully nanhhhh i try it first
    # sudo apt-get remove modemmanager

    sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded -y
    sudo apt-get update
    sudo apt-get install python-serial\
                         openocd \
                         flex \
                         bison\
                         libncurses5-dev \
                         autoconf \
                         texinfo \
                         build-essential \
                         libftdi-dev \
                         libtool \
                         zlib1g-dev \
                         python-empy \
                         gcc-arm-none-eabi -y

}

install_firmware_stack()
{
    mkdir -p $PX4_FIRMWARE_INSTALL_LOCATION
    cd $PX4_FIRMWARE_INSTALL_LOCATION
    git clone https://github.com/PX4/Firmware.git
    cd Firmware
    make posix_sitl_default jmavsim
}

install_gazebo6_bindings()
{
    ### warning!!! this will mess up other gazebo things in the defualt ros listing
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

    # install gazebo 6
    sudo apt-get install ros-indigo-gazebo6-ros-pkgs

    # add a symlink to the catkin_ws
    ln -fs $PWD $HOME/catkin_ws/src/PixhawkFirmware
}


install_rotors_simulator()
{
    cd $PX4_FIRMWARE_INSTALL_LOCATION
    cd ..
    git clone https://github.com/ethz-asl/rotors_simulator

    #add a symlink to catkin ws
    ln -fs $PWD/rotors_simulator $HOME/catkin_ws/src/rotors_simulator

    #get octomap_ros
    sudo apt-get install ros-$ROS_VERSION-octomap_ros
}

install_mav_com()
{
    cd $PX4_FIRMWARE_INSTALL_LOCATION
    cd ..

    git clone https:://github.com/ethz-asl/mav_comm

    #add a symlink to catkin ws
    ln -fs $PWD/mav_com $HOME/catkin_ws/src/mav_com
}


# RUN
install_p4x_deps
px4_install_stuff
install_firmware_stack
install_gazebo6_bindings
install_rotors_simulator
install_mav_com

