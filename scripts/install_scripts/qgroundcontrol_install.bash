#!/bin/sh
set -e  # halt on the first mistake
BUILD_PATH="$HOME/awesomo_build"
QGROUND_FILE="qgroundcontrol.tar.bz2"
DOWNLOAD_URL="http://qgroundcontrol.s3.amazonaws.com/master/$QGROUND_FILE"


install_dependencies()
{
    sudo apt-get install \
        espeak \
        libespeak-dev \
        libudev-dev \
        libsdl1.2-dev
}

install_qgroundcontrol()
{
    cd $BUILD_PATH
    wget $DOWNLOAD_URL
    tar -xvf $QGROUND_FILE
    rm $QGROUND_FILE
    cd $HOME
}


# RUN
install_dependencies
install_qgroundcontrol
