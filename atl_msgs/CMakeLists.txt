CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(atl_msgs)

# DEPENDENCIES
FIND_PACKAGE(
    catkin
    REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
)

# ROS MESSAGE
ADD_MESSAGE_FILES(
    DIRECTORY msgs
    FILES
    AprilTagPose.msg
    LCtrlSettings.msg
    ModelPose.msg
    PCtrlSettings.msg
    PIDSettings.msg
    TCtrlSettings.msg
    VCtrlSettings.msg
)
GENERATE_MESSAGES(DEPENDENCIES std_msgs geometry_msgs)

# CATKIN PACKAGE
CATKIN_PACKAGE(
    CATKIN_DEPENDS
      message_generation
      roscpp
      geometry_msgs
)
