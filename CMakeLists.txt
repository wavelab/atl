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
    atim
    std_msgs
    message_generation
    image_transport
    cv_bridge
    tf
)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(Gazebo REQUIRED)

# LIBS
LINK_DIRECTORIES(
    /usr/lib
    /usr/local/lib
    /usr/local/lib/prototype
    ${GAZEBO_LIBRARY_DIRS}
)

# INCLUDES
INCLUDE_DIRECTORIES(
    ${catkin_INCLUDE_DIRS}
    /usr/local/include
    /usr/local/include/prototype
    /usr/include/eigen3
    /usr/include/opencv2
    /usr/include/flycapture
    ${Boost_INCLUDE_DIR}
    ${GAZEBO_INCLUDE_DIRS}
    ./include
)

# ROS MESSAGE
ADD_MESSAGE_FILES(
    DIRECTORY msg
    FILES
    PositionControllerStats.msg
    KFStats.msg
    KFPlotting.msg
    GimbalSetPoint.msg
)
GENERATE_MESSAGES(DEPENDENCIES std_msgs)

# BUILD AS CATKIN PACKAGE
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES awesomo
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
    DEPENDS system_lib
)

# LIBRARY
ADD_LIBRARY(
    awesomo
    STATIC
    src/util.cpp
    src/ros/ros_util.cpp
    src/controller.cpp
    src/quadrotor.cpp
    src/estimator.cpp
    src/estimator.cpp
    src/planner.cpp
)
SET(AWESOMO_DEPS
    awesomo
    apriltags
    yaml-cpp
    flycapture
    prototype_clients
    prototype_msgs
    prototype
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)
SET(BIN_DEST ${CATKIN_PACKAGE_BIN_DESTINATION})
SET(SHARE_DEST ${CATKIN_PACKAGE_SHARE_DESTINATION})




## ROS
# quadrotor_node
ADD_EXECUTABLE(awesomo_node src/ros/awesomo.cpp)
ADD_DEPENDENCIES(awesomo_node ${PROJECT_NAME}_gencpp)
TARGET_LINK_LIBRARIES(awesomo_node ${AWESOMO_DEPS})
INSTALL(TARGETS awesomo_node RUNTIME DESTINATION ${BIN_DEST})



## UNIT TESTS
# tests/controller_test
ADD_EXECUTABLE(awesomo_controller_test tests/controller_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_controller_test ${AWESOMO_DEPS})
INSTALL(TARGETS awesomo_controller_test RUNTIME DESTINATION ${BIN_DEST})

# tests/quadrotor_test
ADD_EXECUTABLE(awesomo_quadrotor_test tests/quadrotor_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_quadrotor_test ${AWESOMO_DEPS})
INSTALL(TARGETS awesomo_quadrotor_test RUNTIME DESTINATION ${BIN_DEST})

# tests/estimator_test
ADD_EXECUTABLE(awesomo_estimator_test tests/estimator_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_estimator_test ${AWESOMO_DEPS})
INSTALL(TARGETS awesomo_estimator_test RUNTIME DESTINATION ${BIN_DEST})

# tests/planner_test
ADD_EXECUTABLE(awesomo_planner_test tests/planner_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_planner_test ${AWESOMO_DEPS})
INSTALL(TARGETS awesomo_quadrotor_test RUNTIME DESTINATION ${BIN_DEST})

# tests/util_test
ADD_EXECUTABLE(awesomo_util_test tests/util_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_util_test ${AWESOMO_DEPS})
INSTALL(TARGETS awesomo_util_test RUNTIME DESTINATION ${BIN_DEST})



# LAUNCH AND CONFIG FILES
INSTALL(DIRECTORY configs/ DESTINATION ${SHARE_DEST}/configs)
INSTALL(DIRECTORY launch/ DESTINATION ${SHARE_DEST}/launch)
