#!/bin/bash
set -e  # halt on first error
DOWNLOAD_PATH=/usr/local/src
REPO_VER=2016-12-01
REPO_URL="https://april.eecs.umich.edu/media/apriltag/apriltag-$REPO_VER.tgz"

install_apriltags()
{
    # create build directory for atl
    mkdir -p $DOWNLOAD_PATH

    # download and build michigan apriltags
    cd $DOWNLOAD_PATH
    if [ ! -d apriltag_michigan ]; then
        # download and extract
        curl -O $REPO_URL
        tar -xzvf apriltag-$REPO_VER.tgz
        rm apriltag-$REPO_VER.tgz

        # rename directory
        if [ -d apriltags_michigan ]; then
            rm -rf apriltags_michigan
        fi
        sudo mv apriltag-$REPO_VER apriltags_michigan
    fi;
    cd apriltags_michigan
    sudo make && sudo make install
}

# RUN
install_apriltags
