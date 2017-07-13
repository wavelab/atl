#!/bin/bash
set -e  # halt on first error
DOWNLOAD_PATH=/usr/local/src
REPO_URL=https://svn.csail.mit.edu/apriltags
INC_DEST=/usr/local/include/apriltags_mit
LIB_DEST=/usr/local/lib/libapriltags_mit.a
REGEX_STRING='s/AprilTags\//apriltags_mit\//g'


install_dependencies()
{
    # install dependencies
    sudo apt-get install -q -y \
        subversion \
        cmake \
        libeigen3-dev \
        libv4l-dev
}

install_apriltags()
{
    # download and build mit apriltags
    cd $DOWNLOAD_PATH
    if [ ! -d apriltags_mit ]; then
        sudo svn --trust-server-cert --non-interactive co $REPO_URL
        sudo mv apriltags apriltags_mit
    fi
    cd apriltags_mit

    # do some hackery for opencv
    sudo sed -i 's/find_package\(OpenCV\)/find_package\(OpenCV 2.4 REQUIRED\)/g' CMakeLists.txt
    sudo make

    # install
    # as of Aug 31st 2016 they don't have a install target
    # you have to install it manually
    sudo mkdir -p $INC_DEST
    sudo cp -r ./build/include/AprilTags/*.h $INC_DEST
    sudo cp -r ./build/lib/libapriltags.a $LIB_DEST

    # do some hackery and change header file references
    sudo find $INC_DEST -type f -exec sed -i $REGEX_STRING {} +
}

uninstall_apriltags()
{
    sudo rm -rf $INC_DEST
    sudo rm $LIB_DEST
}


# RUN
install_dependencies
install_apriltags
# uninstall_apriltags
