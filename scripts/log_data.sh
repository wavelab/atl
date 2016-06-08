#! /bin/bash
set -e
cd $HOME

rosbag record /awesomo/kf_estimation/stats \
              /awesomo/landing_target/pose \
              /awesomo/position_controller/stats \
              /mavros/imu/data \
              /mavros/global_position/local \
              /mavros/local_position/pose \
              /mavros/local_position/velocity \
              /mavros/px4flow/ground_distance \
              /mavros/extended_state
