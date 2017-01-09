CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(awesomo_msgs)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

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
    GimbalSetPoint.msg
    KFPlot.msg
    KFStats.msg
    PCtrlStats.msg
    AprilTagPose.msg
    PIDSettings.msg
    PCtrlSettings.msg
)
GENERATE_MESSAGES(DEPENDENCIES std_msgs geometry_msgs)

# CATKIN PACKAGE
CATKIN_PACKAGE(
    CATKIN_DEPENDS
      message_generation
      roscpp
      std_msgs
      geometry_msgs
)
