# ATL

**A**utonomous **T**racking and **L**anding (ATL) for multi-rotors.

![ATL in action](https://media.giphy.com/media/3oxHQr4zLbHZSjAJ0Y/giphy.gif)


## Requirements

- ROS Kinetic
- Gazebo 8
- OpenCV 3
- [DJI OSDK](https://github.com/dji-sdk/Onboard-SDK)
- [DJI Onboard-SDK ROS](https://github.com/dji-sdk/Onboard-SDK-ROS)
- PointGrey SDK
- Ximea SDK
- [Michael Kaess's AprilTag implementation](http://people.csail.mit.edu/kaess/apriltags/)

Or if you're very lazy like me you can install all dependencies by

    cd <your catkin workspace>/src
    git clone https://github.com/wavelab/atl
    cd atl/scripts/install
    sudo bash install_deps.bash

This will install all depencies required by ATL.


## Install

The following instructions assumes you have installed ROS.

    cd <your catkin workspace>/src
    git clone https://github.com/wavelab/atl
    cd ..
    catkin build

In short the above navigates to your catkin workspace's `src`, clones `atl`
and finally runs `catkin_make` to build the project, tests and installs configs
and launch files.


## Run Gazebo Simulations

ATL uses Gazebo simulation to test the control and perception, to launch an
instance of a Gazebo simulation along with ATL enter the following commands:

    # Environment variables to tell Gazebo where to look for 3D models and plugins
    # The following assumes you're using `catkin build` instead of `catkin make`
    export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:$HOME/.gazebo
    export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/.gazebo/models
    export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$HOME/.gazebo/plugins

    # Launch Gazebo simulation
    roslaunch atl_ros atl_sim.launch


## License

Licence LGPL License Copyright (C) <2016> Chris Choi, Stan Brown

This program is free software: you can redistribute it and/or modify it under
the terms of the Lesser GNU General Public License as published by the Free
Software Foundation, either version 3 of the License, or (at your option) any
later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program.  If not, see <http://www.gnu.org/licenses/>.
