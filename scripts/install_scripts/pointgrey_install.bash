#!/bin/bash
set -e #halt on first error

# get dir that the script is located in
DIR=$( cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
echo $DIR

install_pointgrey_drivers(){
    # pointgrey driver dependencies
    sudo apt-get install \
        libraw1394-11 \
        libgtkmm-2.4-1c2a \
        libglademm-2.4-1c2a \
        libgtkglextmm-x11-1.2-dev \
        libgtkglextmm-x11-1.2 \
        libusb-1.0-0 \
        -y

    # unzip drivers
    cd $DIR
    cd ../../deps
    unzip flycapture2-2.8.3.1-amd64.zip
    cd flycapture2-2.8.3.1-amd64
    sudo sh install_flycapture.sh
    cd ..
    rm -rf flycapture2-2.8.3.1
}

# install coriander for setting camera configurations
install_coriander(){
    sudo apt-get install coriander -y
}

install_pointgrey_drivers
install_coriander
