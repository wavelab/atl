#!/bin/bash
set -e # halt on first error

PACAKGE=flycapture.2.9.3.13_armhf
POINTGREY_USER=ubuntu

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

    # unzip drivers
    cd ./deps
    unzip $PACAKGE.zip
    cd $PACAKGE

    # copy libraries to /usr/lib
    sudo sh -c "cd lib && cp libflycapture* /usr/lib && cd .."

    # copy headers to /usr/include
    sudo sh -c "cd include && cp *.h /usr/include && cd .."

    # configure permissions to run point grey cameras
    sudo sh flycap2-conf << EOF
$POINTGREY_USER
y
y
y
EOF

    cd ..
    rm -rf $PACAKGE
}

# install coriander for setting camera configurations
install_coriander()
{
    sudo apt-get install coriander -y
}

install_pointgrey_drivers
install_coriander
echo "Installed PointGrey Drivers!"
