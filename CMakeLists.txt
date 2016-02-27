CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(awesomo)

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
)
FIND_PACKAGE(OpenCV)
FIND_PACKAGE(Boost)

# LIBS
LINK_DIRECTORIES(
    /usr/local/lib
    /usr/lib
)

# INCLUDES
INCLUDE_DIRECTORIES(
    ${catkin_INCLUDE_DIRS}
    /usr/local/include
    /usr/include/eigen3
    $(Boost_INCLUDE_DIR)
    ./include
)

# BUILD AS CATKIN PACKAGE
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES awesomo
    CATKIN_DEPENDS roscpp rospy std_msgs
    DEPENDS system_lib
)

# EXECUTABLES
ADD_EXECUTABLE(quadrotor src/quadrotor.cpp)
TARGET_LINK_LIBRARIES(quadrotor ${catkin_LIBRARIES})
INSTALL(
    TARGETS quadrotor
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

ADD_EXECUTABLE(camera src/camera.cpp)
TARGET_LINK_LIBRARIES(
    camera
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    apriltags
    yaml-cpp
)
INSTALL(
    TARGETS camera
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
