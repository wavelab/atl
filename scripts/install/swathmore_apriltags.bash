#!/bin/bash
set -e  # halt on first error
BUILD_PATH="$PWD/awesomo_deps"


install_dependencies()
{
    # install dependencies
    sudo apt-get install -y libcgal-dev
}

install_apriltags()
{
    # create build directory for awesomo
    mkdir -p $BUILD_PATH
    cd $BUILD_PATH

    # donwload and install swathmore apriltags
    git clone https://github.com/swatbotics/apriltags-cpp.git
    cd apriltags-cpp
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE
    make

    # As of Aug 31st 2016 they don't have a install target
    # installing manually
    sudo cp libapriltags.a /usr/local/lib
    sudo mkdir -p /usr/local/include/apriltags/
    sudo cp ../src/*.h /usr/local/include/apriltags/

    # remove apriltags repo
    rm -rf $BUILD_PATH/apriltags-cpp
}

uninstall_apriltags()
{
    sudo rm -rf /usr/local/include/AprilTags
    sudo rm /usr/local/lib/libapriltags.a
}


# RUN
install_dependencies
install_apriltags
# uninstall_apriltags
