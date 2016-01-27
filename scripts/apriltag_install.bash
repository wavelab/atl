#!/bin/bash
set -e  # halt on first error
BUILD_PATH="$HOME/awesomo_build"


install_dependencies()
{
    # install dependencies
    apt-get install -y \
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
    svn co https://svn.csail.mit.edu/apriltags
    cd apriltags && make
    cp -R build/include/AprilTags /usr/include/
    cp -R build/lib/libapriltags.a /usr/lib/

    # remove apriltags repo
    # cd $BUILD_PATH
    # rm -rf apriltags
}


# RUN
install_dependencies
install_apriltags
