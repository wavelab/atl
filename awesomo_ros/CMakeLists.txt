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
    rostest
    std_msgs
    message_generation
    image_transport
    cv_bridge
    tf
    dji_sdk
    awesomo_core
    awesomo_msgs
)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Boost REQUIRED)

# INCLUDES
INCLUDE_DIRECTORIES(
    include
    /usr/include
    /usr/local/include
    /usr/include/eigen3
    /usr/include/opencv2
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

# LIBS
LINK_DIRECTORIES(
    /usr/lib
    /usr/local/lib
    ${catkin_LIBRARY_DIRS}
)

# BUILD AS CATKIN PACKAGE
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        message_runtime
        awesomo_core
        awesomo_msgs
)

# SUBDIRECTORIES
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

# INSTALL LAUNCH FILES
SET(SHARE_DEST ${CATKIN_PACKAGE_SHARE_DESTINATION})
INSTALL(DIRECTORY launch/ DESTINATION ${SHARE_DEST}/launch)
