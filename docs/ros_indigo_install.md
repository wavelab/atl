# Install ROS Indigo

## Add ros packages link

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    
## Add keys

    sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
    
## Update package list

    sudo apt-get update
    
## Base install

    sudo apt-get install ros-indigo-ros-base
    
## Initialize rosdep

    sudo rosdep init
    rosdep update
    
## Setup environment

    echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    
## Get rosinstall 

    sudo apt-get install python-rosinstall
