#!/bin/bash
set -e  # halt on first error
DOWNLOAD_PATH=/usr/local/src
PACKAGE_NAME=XIMEA_Linux_SP.tgz

install_ximea_drivers()
{
    # download ximea driver
    cd $DOWNLOAD_PATH
    if [ ! -f XIMEA_Linux_SP.tgz ]; then
        sudo wget http://www.ximea.com/downloads/recent/XIMEA_Linux_SP.tgz
    fi

    # extract ximea driver
    if [ ! -d ximea_driver ]; then
        sudo tar xzf $PACKAGE_NAME
        sudo mv package ximea_driver
        sudo rm $PACKAGE_NAME
    fi

    # build ximea driver
    cd ximea_driver
    ./install -cam_usb30 << EOF
\n
EOF
}

install_ximea_drivers
echo "Installed Ximea Drivers!"
