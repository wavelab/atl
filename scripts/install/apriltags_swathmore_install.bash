#!/bin/bash
set -e  # halt on first error
BUILD_PATH="$PWD/atl_deps"


install_dependencies()
{
    # install dependencies
    sudo apt-get install -y libcgal-dev
}

install_apriltags()
{
    # create build directory for atl
    mkdir -p $BUILD_PATH
    cd $BUILD_PATH

    # donwload and install swathmore apriltags
    rm -rf apriltags-cpp
    git clone https://github.com/swatbotics/apriltags-cpp.git
    cd apriltags-cpp
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE
    make apriltags

    # As of Aug 31st 2016 they don't have a install target
    # you have to install it manually
    sudo cp libapriltags.a /usr/local/lib/libapriltags_swathmore.a
    sudo mkdir -p /usr/local/include/apriltags_swathmore/
    sudo cp ../src/*.h /usr/local/include/apriltags_swathmore/

    # remove apriltags repo
    rm -rf $BUILD_PATH/apriltags-cpp
}

uninstall_apriltags()
{
    sudo rm -rf /usr/local/include/apriltags_swathmore
    sudo rm /usr/local/lib/libapriltags_swathmore.a
}


# RUN
install_dependencies
install_apriltags
# uninstall_apriltags
