#!/bin/bash
set -e  # halt on first error

PACKAGE_DIR=$PWD/awesomo_deps
PACKAGE_NAME=flycapture2-2.8.3.1-amd64
PACKAGE_EXT=.zip

install_pointgrey_drivers()
{
    # pointgrey driver dependencies
    sudo apt-get install \
        libraw1394-11 \
        libgtkmm-2.4-1c2a \
        libglademm-2.4-1c2a \
        libgtkglextmm-x11-1.2-dev \
        libgtkglextmm-x11-1.2 \
        libusb-1.0-0 \
        -y

    # unzip and install flycapture
    mkdir -p $PACKAGE_DIR
    cd $PACKAGE_DIR
    unzip ${PACKAGE_NAME}${PACAKGE_EXT}
    cd $PACKAGE_NAME
    sh install_flycapture.sh << EOF
y
y
$USER
y
y
n
EOF
    cd ..
    rm -rf $PACAKGE_NAME
}

# install coriander for setting camera configurations
install_coriander()
{
    sudo apt-get install coriander -y
}

install_pointgrey_drivers
install_coriander
echo "Installed PointGrey Drivers!"
