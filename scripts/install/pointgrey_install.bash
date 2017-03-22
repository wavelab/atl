#!/bin/bash
set -e  # halt on first error
UBUNTU_VERSION=`lsb_release --release | cut -f2`

PACKAGE_DIR=$PWD/awesomo_deps

install_pointgrey_drivers()
{
    # pointgrey driver dependencies
    if [ $UBUNTU_VERSION == "16.04" ]; then
        sudo apt-get install -qq \
            libraw1394-11 \
            libgtkmm-2.4-1v5 \
            libglademm-2.4-1v5 \
            libgtkglextmm-x11-1.2-dev \
            libgtkglextmm-x11-1.2 \
            libusb-1.0-0 \
            -y

        # extract and install flycapture
        PACKAGE_NAME="flycapture2-2.10.3.266-amd64"
        PACKAGE_EXT=".tgz"
        mkdir -p $PACKAGE_DIR
        cd $PACKAGE_DIR
        tar -xzf flycapture2-2.10.3.266-amd64.tgz
        cd $PACKAGE_NAME

    elif [ $UBUNTU_VERSION == "14.04" ]; then
        sudo apt-get install -qq \
            libraw1394-11 \
            libgtkmm-2.4-1c2a \
            libglademm-2.4-1c2a \
            libgtkglextmm-x11-1.2-dev \
            libgtkglextmm-x11-1.2 \
            libusb-1.0-0 \
            -y

        # extract and install flycapture
        PACKAGE_NAME="flycapture2-2.8.3.1-amd64"
        PACKAGE_EXT=".zip"
        mkdir -p $PACKAGE_DIR
        cd $PACKAGE_DIR
        unzip ${PACKAGE_NAME}${PACAKGE_EXT}
        cd $PACKAGE_NAME
    fi

    sh install_flycapture.sh << EOF
y
y
$USER
y
y
n
EOF
    cd ..
    rm -rf $PACKAGE_NAME
}

# install coriander for setting camera configurations
install_coriander()
{
    sudo apt-get install coriander -y
}

install_pointgrey_drivers
install_coriander
echo "Installed PointGrey Drivers!"
