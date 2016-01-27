#!/bin/bash
set -e  # halt on the first mistake
ROS_VERSION="indigo"
BUILD_PATH="$HOME/awesomo_build"
CATKIN_WS_PATH="$HOME/awesomo_catkin_ws"

# instructions are taken from here:
# http://dev.px4.io/starting-installing-linux.html

setup_env()
{
    # apparently this is required followed by a logout?
    sudo usermod -a -G dialout $USER

    # create build directory for awesomo
    mkdir -p $BUILD_PATH

    # create catkin_ws for awesomo
    source /opt/ros/$ROS_VERSION/setup.bash
    if [ ! -d "$CATKIN_WS_PATH" ]; then
        mkdir -p $CATKIN_WS_PATH/src
        cd $CATKIN_WS_PATH/src
        catkin_init_workspace
        cd ..
        catkin_make
        cd $HOME
    fi
    source $CATKIN_WS_PATH/devel/setup.bash
}

install_dependencies()
{
    # build essentials
    sudo apt-get update
    sudo apt-get install -y \
        python-argparse \
        git-core \
        wget zip \
        python-empy \
        qtcreator \
        cmake \
        build-essential \
        genromfs

    # simulation tools
    sudo add-apt-repository -y ppa:george-edison55/cmake-3.x
    sudo apt-get update
    sudo apt-get install -y \
        ant \
        protobuf-compiler\
        libeigen3-dev \
        libopencv-dev\
        openjdk-7-jdk \
        openjdk-7-jre \
        clang-3.5 \
        lldb-3.5

    # ros dependencies
    sudo apt-get install -y \
        ros-indigo-mav-msgs \
        ros-indigo-libmavconn \
        ros-$ROS_VERSION-octomap-ros
}

px4_install()
{
    # remove serial modem manager, website says its ok...
    # here it goes then...a
    # acctaully nanhhhh i try it first
    # sudo apt-get remove modemmanager

    sudo add-apt-repository -y ppa:terry.guo/gcc-arm-embedded
    sudo apt-get update
    sudo apt-get install -y \
        python-serial\
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
        gcc-arm-none-eabi
}

install_firmware_stack()
{
    cd $BUILD_PATH
    git clone https://github.com/PX4/Firmware.git
    cd Firmware
    # make posix_sitl_default jmavsim
    make -j2 posix_sitl_default
    cd ..

    # add a symlink to the catkin_ws
    # ln -fs $PWD/Firmware $CATKIN_WS_PATH/src/PixhawkFirmware
    # don't think this is needed

    cd $HOME
}

install_gazebo6_bindings()
{
    ### warning!!! this will mess up other gazebo things in the defualt ros listing
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

    # install gazebo 6
    sudo apt-get update
    sudo apt-get install ros-indigo-gazebo6-ros-pkgs
}

install_rotors_simulator()
{
    cd $BUILD_PATH
    git clone https://github.com/ethz-asl/rotors_simulator

    # add a symlink to catkin ws
    ln -fs $PWD/rotors_simulator $CATKIN_WS_PATH/src/rotors_simulator

    cd $HOME
}

install_mav_com()
{
    cd $BUILD_PATH
    git clone https://github.com/ethz-asl/mav_comm

    # add a symlink to catkin ws
    ln -fs $PWD/mav_comm $CATKIN_WS_PATH/src/mav_comm

    cd $HOME
}

build_catkin_ws()
{
    cd $CATKIN_WS_PATH
    catkin_make --cmake-args -DCONFIG=ros_sitl_simple
}

# RUN
setup_env
install_dependencies
px4_install
install_firmware_stack
install_gazebo6_bindings
install_rotors_simulator
install_mav_com
build_catkin_ws
