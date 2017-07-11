#!/bin/bash
set -e  # exit on first error
DOWNLOAD_PATH="/usr/local/src"
REPO_URL="https://github.com/dji-sdk/Onboard-SDK"

install_djiosdk()
{
    # download DJI Onboard SDK Core
    cd $DOWNLOAD_PATH
    sudo git clone $REPO_URL

    # compile and install
    cd Onboard-SDK
    sudo mkdir build
    cd build
    sudo cmake ..
    sudo make
    sudo make install djiosdk-core
}

# RUN
install_djiosdk
