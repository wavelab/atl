#!/bin/bash
set -e # halt on first error

PACKAGE_DIR=$PWD/deps
NAVIO2_PKG_NAME=navio2-cpp
NAVIO2_CPP_URL=http://github.com/chutsu/$NAVIO2_PKG_NAME


install_navio2_cpp()
{
    cd $PACKAGE_DIR
    git clone $NAVIO2_CPP_URL
    cd $NAVIO2_PKG_NAME
    make && sudo make install
    cd -
}

install_navio2_cpp
