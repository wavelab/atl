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

# LIBRARY
ADD_LIBRARY(
    awesomo
    STATIC
    src/sim.cpp
    src/pid.cpp
    src/util.cpp
)

# EXECUTABLES
# camera
ADD_EXECUTABLE(camera_node src/ros/camera_node.cpp)
TARGET_LINK_LIBRARIES(
    camera_node
    ${catkin_LIBRARIES}
)
INSTALL(
    TARGETS camera_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# quadrotor
ADD_EXECUTABLE(quadrotor src/quadrotor.cpp)
TARGET_LINK_LIBRARIES(quadrotor ${catkin_LIBRARIES} awesomo)
INSTALL(
    TARGETS quadrotor
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# camera
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

# sim_test
ADD_EXECUTABLE(sim_test tests/sim_test.cpp)
TARGET_LINK_LIBRARIES(sim_test awesomo)
INSTALL(TARGETS sim_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# pid_test
ADD_EXECUTABLE(pid_test tests/pid_test.cpp)
TARGET_LINK_LIBRARIES(pid_test awesomo)
INSTALL(TARGETS pid_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# util_test
ADD_EXECUTABLE(util_test tests/util_test.cpp)
TARGET_LINK_LIBRARIES(util_test awesomo)
INSTALL(TARGETS util_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
