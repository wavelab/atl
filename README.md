# ATL

**A**utonomous **T**racking and **L**anding (ATL) for multi-rotors.


## Install

The following instructions assumes you have installed ROS.

    cd <your catkin workspace>/src
    git clone https://github.com/wavelab/atl
    cd ..
    catkin_make
    catkin_make tests
    catkin_make install

In short the above navigates to your catkin workspace's `src`, clones `atl`
and finally runs `catkin_make` to build the project, tests and installs configs
and launch files.


## Run Gazebo Simulations

ATL uses Gazebo simulation to test the control and perception, to launch an
instance of a Gazebo simulation along with ATL enter the following commands:

    # Environment variables to tell Gazebo where to look for 3D models and plugins
    export CATKIN_WS=<your catkin workspace>
    export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:${CATKIN_WS}/install/share/atl_gazebo/
    export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:${CATKIN_WS}/install/share/atl_gazebo/models
    export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${CATKIN_WS}/install/share/atl_gazebo/plugins

    # Launch Gazebo simulation
    roslaunch atl_ros atl_sim.launch


## ROS Node Tests

Once you have performed the above instructions you can run ROS node tests via:

    catkin_make run_tests

Or alternatively you can launch individual tests via:

    rostest atl_ros apriltag_test.launch
    rostest atl_ros camera_test.launch
    rostest atl_ros estimator_test.launch
    rostest atl_ros gimbal_test.launch


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
