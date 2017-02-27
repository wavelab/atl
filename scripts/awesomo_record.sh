#!/bin/sh
rosbag record \
    /awesomo/quadrotor/pose/local \
    /awesomo/quadrotor/velocity/local \
    /awesomo/imu \
    /awesomo/apriltag/target \
    /awesomo/apriltag/target/position/inertial \
    /awesomo/apriltag/target/position/body \
    /awesomo/apriltag/target/position/body_encoders \
    /awesomo/apriltag/target/yaw/inertial \
    /awesomo/apriltag/target/yaw/body \
    /awesomo/gimbal/setpoint/attitude \
    /awesomo/gimbal/position/inertial \
    /awesomo/gimbal/frame/orientation/inertial \
    /awesomo/gimbal/joint/orientation/inertial \
    /awesomo/gimbal/joint/orientation/body \
    /awesomo/estimate/landing_target/detected \
    /awesomo/estimate/landing_target/position/inertial \
    /awesomo/estimate/landing_target/position/body \
    /awesomo/estimate/landing_target/velocity/inertial \
    /awesomo/estimate/landing_target/velocity/body \
    /dji_sdk/local_position \
    /dji_sdk/global_position \
    /dji_sdk/attitude_quaternion \
    /dji_sdk/compass \
    /dji_sdk/velocity
