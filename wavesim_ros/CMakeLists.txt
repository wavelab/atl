CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)
PROJECT(wavesim_ros)

# COMPILER SETTINGS
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
IF (CMAKE_COMPILER_IS_GNUCXX)
  ADD_DEFINITIONS(-std=gnu++11)
ENDIF()

# DEPENDENCIES
FIND_PACKAGE(gazebo REQUIRED)
FIND_PACKAGE(
  catkin
  REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    image_transport
    cv_bridge
    OpenCV
    dji_sdk
    wavesim_gazebo
)

# GENERATE CUSTOM ROS MSGS
ADD_MESSAGE_FILES(
  DIRECTORY msgs
  FILES
  ModelPose.msg
)
GENERATE_MESSAGES(DEPENDENCIES std_msgs geometry_msgs)

# INCLUDES
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(/usr/include/eigen3)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GAZEBO_INCLUDE_DIRS})

# BUILD AS CATKIN PACKAGE
CATKIN_PACKAGE(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_runtime
    wavesim_gazebo
)

# LIBS
LINK_DIRECTORIES(${catkin_LIBRARY_DIRS})
LINK_DIRECTORIES(${GAZEBO_LIBRARY_DIRS})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

# ROS UTILS
ADD_LIBRARY(
    wavesim_ros_utils
    STATIC
    src/utils/node.cpp
    src/utils/utils.cpp
)

# ROS NODES
SET(
    ROS_DEPS
    wavesim_ros_utils
    wavesim_gazebo
    wave
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp
    image_transport
    cv_bridge
    dji_sdk_lib
)
ADD_EXECUTABLE(wavesim_camera_node src/nodes/camera_node.cpp)
ADD_DEPENDENCIES(wavesim_camera_node wavesim_ros_utils)
TARGET_LINK_LIBRARIES(wavesim_camera_node ${ROS_DEPS})

ADD_EXECUTABLE(wavesim_df_camera_node src/nodes/df_camera_node.cpp)
ADD_DEPENDENCIES(wavesim_df_camera_node wavesim_ros_utils)
TARGET_LINK_LIBRARIES(wavesim_df_camera_node ${ROS_DEPS})

ADD_EXECUTABLE(wavesim_dji_quadrotor_node src/nodes/dji_quadrotor_node.cpp)
ADD_DEPENDENCIES(wavesim_dji_quadrotor_node wavesim_ros_utils)
TARGET_LINK_LIBRARIES(wavesim_dji_quadrotor_node ${ROS_DEPS})

ADD_EXECUTABLE(wavesim_gimbal_node src/nodes/gimbal_node.cpp)
ADD_DEPENDENCIES(wavesim_gimbal_node wavesim_ros_utils)
TARGET_LINK_LIBRARIES(wavesim_gimbal_node ${ROS_DEPS})

ADD_EXECUTABLE(wavesim_lz_node src/nodes/lz_node.cpp)
ADD_DEPENDENCIES(wavesim_lz_node wavesim_ros_utils)
TARGET_LINK_LIBRARIES(wavesim_lz_node ${ROS_DEPS})

ADD_EXECUTABLE(wavesim_px4_quadrotor_node src/nodes/px4_quadrotor_node.cpp)
ADD_DEPENDENCIES(wavesim_px4_quadrotor_node wavesim_ros_utils)
TARGET_LINK_LIBRARIES(wavesim_px4_quadrotor_node ${ROS_DEPS})

# ADD_EXECUTABLE(world_node nodes/world_node.cpp)
# ADD_DEPENDENCIES(world_node wavesim_ros_utils)
# TARGET_LINK_LIBRARIES(world_node ${ROS_DEPS})


# TESTS
SET(
    WAVESIM_ROS_TEST_DEPS
    wavesim_ros_utils
    wavesim_gazebo_clients
    wavesim_gazebo_msgs
    wave
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${roscpp_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp
    gtest
    pthread
)

ADD_EXECUTABLE(ros_utils_test tests/ros_utils_test.cpp)
TARGET_LINK_LIBRARIES(ros_utils_test ${WAVESIM_ROS_TEST_DEPS})

# ADD_EXECUTABLE(ros_node_test tests/ros_node_test.cpp)
# TARGET_LINK_LIBRARIES(ros_node_test ${WAVESIM_ROS_TEST_DEPS})



# INSTALL LAUNCH FILES
SET(SHARE_DEST ${CATKIN_PACKAGE_SHARE_DESTINATION})
INSTALL(DIRECTORY launch/ DESTINATION ${SHARE_DEST}/launch)
