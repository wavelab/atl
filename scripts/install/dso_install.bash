#!/bin/sh
set -e
ROS_DISTRO="kinetic"
CATKIN_WS=$HOME/catkin_ws/


install_pangolin()
{
    # clone repo
    cd /usr/local/src
    if [ ! -d Pangolin ]; then
        git clone https://github.com/stevenlovegrove/Pangolin
    fi

    # install dependencies
    sudo apt-get install -y -qq libglew-dev

    # build
    cd Pangolin
    mkdir -p build
    cd build
    cmake ..
    make -j
}

install_ziplib()
{
	# install ziplib
	sudo apt-get install -y -qq zlib1g-dev

	cd $CATKIN_WS/src/dso
	cd thirdparty

    if [ ! -d libzip-1.1.1 ]; then
        tar -zxvf libzip-1.1.1.tar.gz
    fi

	cd libzip-1.1.1/
	./configure
	make
	sudo make install
	sudo cp lib/zipconf.h /usr/local/include/zipconf.h
}

install_dependencies()
{
	# dependencies available from apt
    sudo apt-get install -y \
        git \
        cmake \
        libeigen3-dev \
        libsuitesparse-dev \
        libboost-all-dev

	# source based dependencies
	install_pangolin
	install_ziplib
}

install_dso()
{
    cd $CATKIN_WS/src
    if [ ! -d dso ]; then
        git clone https://github.com/JakobEngel/dso
    fi

	# initialize submodules
	cd dso
	git submodule update --init

	# install dependencies
	install_dependencies

	# build dso
    cd $CATKIN_WS
    source /opt/ros/kinetic/setup.bash
    catkin_make
}

# RUN
install_dso
