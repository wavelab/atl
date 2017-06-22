#!/bin/sh
set -e  # exit on first error

install_protobuf()
{
    sudo apt-get install -y protobuf-compiler
}


# main
install_protobuf
