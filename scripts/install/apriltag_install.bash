#!/bin/bash
set -e  # halt on first error
BUILD_PATH="/tmp/build"


install_dependencies()
{
    # install dependencies
    sudo apt-get install -y \
        subversion \
        cmake \
        libopencv-dev \
        libeigen3-dev \
        libv4l-dev
}

install_apriltags()
{
    # create build directory for awesomo
    mkdir -p $BUILD_PATH

    cd $BUILD_PATH
    svn --trust-server-cert --non-interactive co https://svn.csail.mit.edu/apriltags
    cd apriltags
    export BUILD_PREFIX=$BUILD_PATH/apriltags/build
    echo $BUILD_PREFIX
    mkdir -p $BUILD_PREFIX
    make
    sudo cp -r ./build/include/AprilTags /usr/include/
    sudo cp -r ./build/lib/libapriltags.a /usr/lib/

    # remove apriltags repo
    cd $BUILD_PATH
    rm -rf apriltags
}

uninstall_apriltags()
{
    sudo rm -rf /usr/include/AprilTags
    sudo rm /usr/lib/libapriltags.a
}


# RUN
install_dependencies
install_apriltags
# uninstall_apriltags
