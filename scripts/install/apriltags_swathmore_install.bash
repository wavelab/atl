#!/bin/bash
set -e  # halt on first error
DOWNLOAD_PATH=/usr/local/src


install_dependencies()
{
    sudo apt-get install -y -qq libcgal-dev
}

install_apriltags()
{
    # donwload swathmore apriltags
    cd $DOWNLOAD_PATH
    if [ ! -d apriltags_swathmore ]; then
        sudo git clone https://github.com/swatbotics/apriltags-cpp.git
        sudo mv apriltags-cpp apriltags_swathmore
    fi

    # build library
    cd apriltags_swathmore
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=RELEASE
    make apriltags

    # install
    # as of Aug 31st 2016 they don't have a install target
    # you have to install it manually
    sudo cp libapriltags.a /usr/local/lib/libapriltags_swathmore.a
    sudo mkdir -p /usr/local/include/apriltags_swathmore/
    sudo cp ../src/*.h /usr/local/include/apriltags_swathmore/
}

uninstall_apriltags()
{
    sudo rm -rf /usr/local/include/apriltags_swathmore
    sudo rm /usr/local/lib/libapriltags_swathmore.a
}


# RUN
install_dependencies
install_apriltags
# uninstall_apriltags
