#!/bin/sh
set -e  # exit on first error

export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$PWD/prototype_gazebo/src/models
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$PWD/build/prototype_gazebo/plugins

# gazebo prototype_gazebo/tests/worlds/empty_test.world --verbose
# gazebo prototype_gazebo/tests/worlds/camera_test.world --verbose
# gazebo prototype_gazebo/tests/worlds/quadrotor_test.world --verbose
# gazebo prototype_gazebo/tests/worlds/gimbal_test.world --verbose
# gazebo prototype_gazebo/tests/worlds/world_test.world --verbose

# gazebo prototype_gazebo/src/worlds/simulation_camera.world --verbose
# gzserver prototype_gazebo/src/worlds/simulation.world --verbose
