#!/bin/sh
export CLASSPATH=$CLASSPATH:/usr/share/java/gluegen-rt.jar:/usr/local/share/java/lcm.jar:$HOME/april/java/april.jar:./
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/april/lib
alias java='java -ea -server'


install_dependencies()
{
    # install dependencies
    apt-get install -y \
        git-core \
        ant subversion \
        gtk-doc-tools \
        libglib2.0-dev \
        libusb-1.0-0-dev \
        gv \
        libncurses-dev \
        openjdk-7-jdk \
        autopoint \
        libgl1-mesa-dev \
        libpng12-dev \
        libdc1394-22-dev
}

# install LCM
install_lcm()
{
    cd $HOME
    git clone https://code.google.com/p/lcm lcm
    cd lcm
    ./bootstrap.sh
    ./configure
    make
    sudo make install
    rm -rf $HOME/lcm
}

# install april toolkit
install_april()
{
    cd $HOME
    git clone git://april.eecs.umich.edu/home/git/april.git
    cd april/src
    make all
    cd $HOME
    cd april/java
    ant
    rm -rf $HOME/april
}

# RUN
# install_dependencies
# install_lcm
# install_april

# cd $HOME
# # git clone https://github.com/liuliu/ccv.git
# cd ccv/lib
# ./configure && make
# clang \
#     -L"$HOME/ccv/lib" \
#     -I"$HOME/ccv/lib" sift.c \
#     -lccv `cat $HOME/ccv/lib/.deps`
cd $HOME
# rm -rf ccv
