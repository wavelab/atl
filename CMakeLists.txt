CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(awesomo)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

# OPENMP
FIND_PACKAGE(OpenMP REQUIRED)
if (OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

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
    /usr/include/flycapture
    $(Boost_INCLUDE_DIR)
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
    libawesomo
    STATIC
    src/util.cpp
    src/ros/ros_util.cpp
    src/camera_mount.cpp
    src/controller.cpp
    src/quadrotor.cpp
    src/estimator.cpp
    src/estimator.cpp
    src/planner.cpp
    src/sbgc.cpp
)
SET(AWESOMO_DEPS
    libawesomo
    apriltags
    yaml-cpp
    flycapture
    serial
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
SET(BIN_DEST ${CATKIN_PACKAGE_BIN_DESTINATION})
SET(SHARE_DEST ${CATKIN_PACKAGE_SHARE_DESTINATION})




## ROS NODES
# quadrotor_node
ADD_EXECUTABLE(awesomo src/ros/awesomo.cpp)
ADD_DEPENDENCIES(awesomo ${PROJECT_NAME}_gencpp)
TARGET_LINK_LIBRARIES(awesomo ${AWESOMO_DEPS})
INSTALL(TARGETS awesomo RUNTIME DESTINATION BIN_DEST)



## UNIT TESTS
# tests/controller_test
ADD_EXECUTABLE(awesomo_controller_test tests/controller_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_controller_test libawesomo yaml-cpp)
INSTALL(TARGETS awesomo_controller_test RUNTIME DESTINATION BIN_DEST)

# tests/quadrotor_test
ADD_EXECUTABLE(awesomo_quadrotor_test tests/quadrotor_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_quadrotor_test ${AWESOMO_DEPS})
INSTALL(TARGETS awesomo_quadrotor_test RUNTIME DESTINATION BIN_DEST)

# tests/estimator_test
ADD_EXECUTABLE(awesomo_estimator_test tests/estimator_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_estimator_test ${AWESOMO_DEPS})
INSTALL(TARGETS awesomo_estimator_test RUNTIME DESTINATION BIN_DEST)

# tests/planner_test
ADD_EXECUTABLE(awesomo_planner_test tests/planner_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_planner_test ${AWESOMO_DEPS})
INSTALL(TARGETS awesomo_quadrotor_test RUNTIME DESTINATION BIN_DEST)

# tests/sbgc_test
ADD_EXECUTABLE(awesomo_sbgc_test tests/sbgc_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_sbgc_test ${AWESOMO_DEPS})
INSTALL(TARGETS awesomo_quadrotor_test RUNTIME DESTINATION BIN_DEST)

# tests/util_test
ADD_EXECUTABLE(awesomo_util_test tests/util_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_util_test libawesomo)
INSTALL(TARGETS awesomo_util_test RUNTIME DESTINATION BIN_DEST)

# tests/camera_mount_config_test
ADD_EXECUTABLE(awesomo_camera_mount_test tests/camera_mount_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_camera_mount_test libawesomo yaml-cpp)
INSTALL(TARGETS awesomo_camera_mount_test RUNTIME DESTINATION BIN_DEST)



# LAUNCH AND CONFIG FILES
INSTALL(DIRECTORY configs/ DESTINATION SHARE_DEST/configs)
INSTALL(DIRECTORY launch/ DESTINATION SHARE_DEST/launch)
