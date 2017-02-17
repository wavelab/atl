#!/bin/bash
set -e  # halt on first error

PACKAGE_DIR=$PWD/awesomo_deps
PACKAGE_NAME=XIMEA_Linux_SP.tgz

install_ximea_drivers()
{
    mkdir -p $PACKAGE_DIR
    cd $PACKAGE_DIR
    wget http://www.ximea.com/downloads/recent/XIMEA_Linux_SP.tgz
    tar xzf $PACKAGE_NAME
    mv package ximea_driver
    cd ximea_driver
    ./install -cam_usb30 << EOF
\n
EOF
}

install_ximea_drivers
echo "Installed Ximea Drivers!"
