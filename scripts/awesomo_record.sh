#!/bin/sh
rosbag record \
    /awesomo/quadrotor/pose/local \
    /awesomo/quadrotor/velocity/local \
    /awesomo/apriltag/target \
    /awesomo/apriltag/target/position/body \
    /awesomo/apriltag/target/position/inertial \
    /awesomo/apriltag/target/yaw/body \
    /awesomo/apriltag/target/yaw/inertial \
    /awesomo/gimbal/setpoint/attitude \
    /awesomo/gimbal/position/inertial \
    /awesomo/gimbal/frame/orientation/inertial \
    /awesomo/gimbal/joint/orientation/inertial \
    /awesomo/estimate/landing_target/detected \
    /awesomo/estimate/landing_target/position/inertial \
    /awesomo/estimate/landing_target/position/body \
    /awesomo/estimate/landing_target/velocity/inertial \
    /awesomo/estimate/landing_target/velocity/body \
    /dji_sdk/local_position \
    /dji_sdk/global_position \
    /dji_sdk/attitude_quaternion \
    /dji_sdk/velocity
