#!/bin/sh
# rosbag record \
#     /atl/quadrotor/pose/local \
#     /atl/quadrotor/velocity/local \
#     /atl/imu \
#     /atl/apriltag/target \
#     /atl/apriltag/target/position/inertial \
#     /atl/apriltag/target/position/body \
#     /atl/apriltag/target/position/body_encoders \
#     /atl/apriltag/target/yaw/inertial \
#     /atl/apriltag/target/yaw/body \
#     /atl/gimbal/setpoint/attitude \
#     /atl/gimbal/position/inertial \
#     /atl/gimbal/frame/orientation/inertial \
#     /atl/gimbal/joint/orientation/inertial \
#     /atl/gimbal/joint/orientation/body \
#     /atl/estimate/landing_target/detected \
#     /atl/estimate/landing_target/position/inertial \
#     /atl/estimate/landing_target/position/body \
#     /atl/estimate/landing_target/velocity/inertial \
#     /atl/estimate/landing_target/velocity/body \
#     /atl/lz/pose \
#     /dji_sdk/local_position \
#     /dji_sdk/global_position \
#     /dji_sdk/attitude_quaternion \
#     /dji_sdk/compass \
#     /dji_sdk/velocity

rosbag record \
    /atl/quadrotor/pose/local \
    /atl/quadrotor/velocity/local \
    /atl/gimbal_camera/image/compressed \
    /dji_sdk/flight_control_info \
    /dji_sdk/flight_status \
    /dji_sdk/rc_channels \
    /dji_sdk/time_stamp \
    /dji_sdk/local_position \
    /dji_sdk/global_position \
    /dji_sdk/attitude_quaternion \
    /dji_sdk/compass \
    /dji_sdk/velocity \
    /dji_sdk/acceleration
