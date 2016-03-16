#!/bin/bash
set -e #halt on first error

# get dir that the script is located in
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
install_pointgrey_drivers(){
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
    sudo apt-get install coriander
}

install_pointgrey_drivers
install_coriander


