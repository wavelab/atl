#! /bin/bash
set -e
cd $HOME

# rosbag record /awesomo/kf_estimation/stats \
#               /awesomo/landing_target/pose \
#               /awesomo/position_controller/stats \
rosbag record /mavros/imu/data \
              /mavros/global_position/local \
              /mavros/local_position/pose \
              /mavros/local_position/velocity \
              /mavros/extended_state
