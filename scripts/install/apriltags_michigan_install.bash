#!/bin/bash
set -e  # halt on first error
BUILD_PATH="$PWD/awesomo_deps"
REPO_VER=2016-12-01
REPO_URL="https://april.eecs.umich.edu/media/apriltag/apriltag-$REPO_VER.tgz"

install_apriltags()
{
    # create build directory for awesomo
    mkdir -p $BUILD_PATH

    # download and build michigan apriltags
    cd $BUILD_PATH
    curl -O $REPO_URL
    tar -xzvf apriltag-$REPO_VER.tgz
    cd apriltag-$REPO_VER
    make && sudo make install

    # clean up
    cd ..
    rm apriltag-$REPO_VER.tgz
    rm -rf apriltag-$REPO_VER
}

# RUN
install_apriltags
