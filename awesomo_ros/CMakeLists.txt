CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(awesomo_ros)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

# DEPENDENCIES
FIND_PACKAGE(
    catkin
    REQUIRED COMPONENTS
    roscpp
    rospy
    atim
    std_msgs
    message_generation
    image_transport
    cv_bridge
    tf
    awesomo_core
    awesomo_msgs
)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Boost REQUIRED)

# LIBS
LINK_DIRECTORIES(
    /usr/lib
    /usr/local/lib
)

# INCLUDES
INCLUDE_DIRECTORIES(
    ${catkin_INCLUDE_DIRS}
    include
    /usr/include
    /usr/local/include
    /usr/include/eigen3
    /usr/include/opencv2
    ${Boost_INCLUDE_DIR}
)

# BUILD AS CATKIN PACKAGE
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES awesomo_core
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
    DEPENDS system_lib
)

SET(DEPS
    awesomo_core
    ${catkin_LIBRARIES}
    yaml-cpp
)

# ROS NODES
SET(BIN_DEST ${CATKIN_PACKAGE_BIN_DESTINATION})
ADD_EXECUTABLE(awesomo_node src/ros/awesomo_node.cpp)
ADD_DEPENDENCIES(awesomo_node ${PROJECT_NAME}_gencpp awesomo_core)
TARGET_LINK_LIBRARIES(awesomo_node ${DEPS})
INSTALL(TARGETS awesomo_node RUNTIME DESTINATION ${BIN_DEST})

# INSTALL LAUNCH FILES
SET(SHARE_DEST ${CATKIN_PACKAGE_SHARE_DESTINATION})
INSTALL(DIRECTORY launch/ DESTINATION ${SHARE_DEST}/launch)
