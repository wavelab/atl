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
    message_generation
)

# ROS MESSAGE
ADD_MESSAGE_FILES(
    DIRECTORY msgs
    FILES
    PositionControllerStats.msg
    KFStats.msg
    KFPlotting.msg
    GimbalSetPoint.msg
)
GENERATE_MESSAGES(DEPENDENCIES std_msgs)

CATKIN_PACKAGE(
    CATKIN_DEPENDS message_generation roscpp std_msgs geometry_msgs
)
