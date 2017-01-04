#!/bin/bash
set -e  # halt on first error
BUILD_PATH="$PWD/awesomo_deps"
REPO_URL=https://svn.csail.mit.edu/apriltags
INC_DEST=/usr/local/include/apriltags_mit
LIB_DEST=/usr/local/lib/libapriltags_mit.a
REGEX_STRING='s/AprilTags\//apriltags_mit\//g'


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

    # download and build mit apriltags
    cd $BUILD_PATH
    rm -rf apriltags
    svn --trust-server-cert --non-interactive co $REPO_URL
    cd apriltags
    export BUILD_PREFIX=$BUILD_PATH/apriltags/build
    mkdir -p $BUILD_PREFIX
    make

    # install
    # as of Aug 31st 2016 they don't have a install target
    # you have to install it manually
    mkdir -p $INC_DEST
    sudo cp -r ./build/include/AprilTags/*.h $INC_DEST
    sudo cp -r ./build/lib/libapriltags.a $LIB_DEST

    # do some hackery and change header file references
    sudo find $INC_DEST -type f -exec sed -i $REGEX_STRING {} +

    # remove apriltags repo
    cd $BUILD_PATH
    rm -rf apriltags
}

uninstall_apriltags()
{
    sudo rm -rf $INC_DEST
    sudo rm $LIB_DEST
}


# RUN
install_dependencies
install_apriltags
# uninstall_apriltags
