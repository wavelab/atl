CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)

# SETTINGS
SET(PLUGIN_DEST /usr/local/lib/wavesim/plugins)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/ros)

# ROS UTILS
ADD_LIBRARY(
    wavesim_ros_utils
    STATIC
    utils/node.cpp
    utils/utils.cpp
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
ADD_EXECUTABLE(camera_node nodes/camera_node.cpp)
ADD_DEPENDENCIES(camera_node wavesim_ros_utils)
TARGET_LINK_LIBRARIES(camera_node ${ROS_DEPS})

ADD_EXECUTABLE(df_camera_node nodes/df_camera_node.cpp)
ADD_DEPENDENCIES(df_camera_node wavesim_ros_utils)
TARGET_LINK_LIBRARIES(df_camera_node ${ROS_DEPS})

ADD_EXECUTABLE(dji_quadrotor_node nodes/dji_quadrotor_node.cpp)
ADD_DEPENDENCIES(dji_quadrotor_node wavesim_ros_utils)
TARGET_LINK_LIBRARIES(dji_quadrotor_node ${ROS_DEPS})

ADD_EXECUTABLE(gimbal_node nodes/gimbal_node.cpp)
ADD_DEPENDENCIES(gimbal_node wavesim_ros_utils)
TARGET_LINK_LIBRARIES(gimbal_node ${ROS_DEPS})

ADD_EXECUTABLE(lz_node nodes/lz_node.cpp)
ADD_DEPENDENCIES(lz_node wavesim_ros_utils)
TARGET_LINK_LIBRARIES(lz_node ${ROS_DEPS})

ADD_EXECUTABLE(px4_quadrotor_node nodes/px4_quadrotor_node.cpp)
ADD_DEPENDENCIES(px4_quadrotor_node wavesim_ros_utils)
TARGET_LINK_LIBRARIES(px4_quadrotor_node ${ROS_DEPS})

# ADD_EXECUTABLE(world_node nodes/world_node.cpp)
# ADD_DEPENDENCIES(world_node wavesim_ros_utils)
# TARGET_LINK_LIBRARIES(world_node ${ROS_DEPS})
