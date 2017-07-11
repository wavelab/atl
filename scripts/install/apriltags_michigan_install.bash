#!/bin/bash
set -e  # halt on first error
DOWNLOAD_PATH=/usr/local/src
REPO_VER=2016-12-01
REPO_URL="https://april.eecs.umich.edu/media/apriltag/apriltag-$REPO_VER.tgz"

install_dependencies()
{
    sudo apt-get install -qq -y libopencv-dev
}

install_apriltags()
{
    install_dependencies

    # create build directory for atl
    mkdir -p $DOWNLOAD_PATH

    # download and build michigan apriltags
    cd $DOWNLOAD_PATH
    if [ ! -d apriltag_michigan ]; then
        sudo curl -O $REPO_URL
        sudo tar -xzvf apriltag-$REPO_VER.tgz
        sudo mv apriltag-$REPO_VER apriltags_michigan
        sudo rm apriltag-$REPO_VER.tgz
    fi;
    cd apriltags_michigan
    sudo make && sudo make install

    # clean up
    cd ..
}

# RUN
install_apriltags
