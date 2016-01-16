#!/bin/sh

install_dependencies()
{
    # install dependencies
    apt-get install -y \
        subversion \
        cmake \
        libopencv-dev \
        libeigen3-dev \
        libv4l-dev
}

install_apriltags()
{
    cd $HOME
    svn co https://svn.csail.mit.edu/apriltags
    cd apriltags && make
    cp -R build/include/AprilTags /usr/include/
    cp -R build/lib/libapriltags.a /usr/lib/
    cd $HOME
    rm -rf apriltags
}


# RUN
install_dependencies
install_apriltags
