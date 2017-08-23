#!/bin/bash
set -e  # halt on first error
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_DIR=$(dirname "$SCRIPT_DIR")
PACKAGE_DIR="${REPO_DIR}/../atl_deps"

install_pointgrey_x86_drivers()
{
    # settings
    PACKAGE_NAME="flycapture2-2.11.3.121-amd64"
    PACKAGE_EXT=".tgz"

    # pointgrey driver dependencies
    sudo apt-get install -qq \
        libraw1394-11 \
        libglade2-dev \
        libglademm-2.4-dev \
        libgtkmm-2.4-1v5 \
        libglademm-2.4-1v5 \
        libgtkglextmm-x11-1.2-dev \
        libgtkglextmm-x11-1.2 \
        libusb-1.0-0 \
        -y

    # extract and install flycapture
    cd $PACKAGE_DIR
    tar -xzf flycapture2-2.11.3.121-amd64-pkg.tgz
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
    rm -rf $PACKAGE_NAME
}

install_pointgrey_arm_drivers()
{
    # settings
    ARM_DRIVER=flycapture.2.9.3.13_armhf

    # pointgrey driver dependencies
    sudo apt-get install -qq -y \
        libraw1394-11 \
        libgtkmm-2.4-1v5 \
        libglademm-2.4-1v5 \
        libgtkglextmm-x11-1.2-dev \
        libgtkglextmm-x11-1.2 \
        libusb-1.0-0

    # unzip drivers
    cd $PACKAGE_DIR
    if [ ! -d $ARM_DRIVER ]; then
        unzip $ARM_DRIVER.zip
    fi
    cd $ARM_DRIVER

    # copy libraries to /usr/lib
    sudo sh -c "cd lib && cp libflycapture* /usr/lib && cd .."

    # copy headers to /usr/include
    sudo mkdir -p /usr/include/flycapture
    sudo sh -c "cd include && cp *.h /usr/include/flycapture && cd .."

    # configure permissions to run point grey cameras
    sudo sh flycap2-conf << EOF
$USER
y
y
y
EOF

    cd ..
    rm -rf $ARM_DRIVER
}

# install coriander for setting camera configurations
install_coriander()
{
    sudo apt-get install -y -qq coriander 
}

# RUN
ARCH_TYPE=`uname -m`
if [ ${ARCH_TYPE} == 'x86_64' ]; then
    install_pointgrey_x86_drivers
elif [ ${ARCH_TYPE} == 'armv7l' ]; then
    install_pointgrey_arm_drivers
else
    echo "Unsupported ARCH_TYPE [$ARCH_TYPE]!";
    exit -1;
fi

install_coriander
echo "Installed PointGrey Drivers!"
