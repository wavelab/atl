set -e  # halt on the first mistake
BUILD_PATH="$PWD"
GIT_REPO_URL="https://github.com/wjwwood/serial.git"


download_repo()
{
    git clone $GIT_REPO_URL
}

install_serial()
{
    cd serial
    # change default install location
    sed -i "s#/tmp/usr/local#/usr/local#" Makefile
    make
    sudo make install
}

remove_repo()
{
    cd $BUILD_PATH
    rm -rf serial
}

uninstall_serial()
{
    cd serial
    # change default install location
    sed -i "s#/tmp/usr/local#/usr/local#" Makefile
    make
    sudo make uninstall
}


# RUN
download_repo
install_serial
remove_repo

#UNINSTALL
# download_repo
# uninstall_serial
# remove_repo
