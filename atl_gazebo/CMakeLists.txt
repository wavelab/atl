CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)
PROJECT(atl_gazebo)

# COMPILER SETTINGS
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC")
IF (CMAKE_COMPILER_IS_GNUCXX)
  ADD_DEFINITIONS(-std=gnu++11)
ENDIF()

# DEPENDENCIES
FIND_PACKAGE(gazebo REQUIRED)
FIND_PACKAGE(OpenCV 3.0 REQUIRED)
FIND_PACKAGE(Protobuf REQUIRED)
FIND_PACKAGE(catkin
             REQUIRED COMPONENTS
               roscpp
               rospy
               std_msgs
               geometry_msgs
               message_runtime
               image_transport
               cv_bridge
               atl_core
               atl_ros
               atl_msgs)

# INCLUDES
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(/usr/include/eigen3)
INCLUDE_DIRECTORIES(SYSTEM ${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GAZEBO_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/msgs)

# LIBS
LINK_DIRECTORIES(/usr/lib)
LINK_DIRECTORIES(/usr/local/lib)
LINK_DIRECTORIES(${catkin_LIBRARY_DIRS})
LINK_DIRECTORIES(${GAZEBO_LIBRARY_DIRS})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})
LINK_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/gazebo)
LINK_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/lib)

# CATKIN
CATKIN_PACKAGE(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS
                 roscpp
                 rospy
                 std_msgs
                 geometry_msgs
                 message_runtime
                 image_transport
                 cv_bridge
                 atl_core
                 atl_ros
                 atl_msgs)

# MSGS
SET(PROTOBUF_IMPORT_DIRS)
FOREACH(ITR ${GAZEBO_INCLUDE_DIRS})
	IF(ITR MATCHES ".*gazebo-[0-9.]+$")
		SET(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
	ENDIF()
ENDFOREACH()
SET(GAZEBO_MSGS
		msgs/attitude_setpoint.proto
		msgs/model_pose.proto
		msgs/position_setpoint.proto
		msgs/rpy_pose.proto
		msgs/velocity_setpoint.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${GAZEBO_MSGS})
ADD_LIBRARY(atl_gazebo_msgs STATIC ${PROTO_SRCS})
TARGET_LINK_LIBRARIES(atl_gazebo_msgs ${PROTOBUF_LIBRARY})


# BRIDGE
SET(ATL_GAZEBO_DEPS
    atl_core
    atl_ros_utils
    atl_gazebo_clients
    atl_gazebo_msgs
    ${GAZEBO_LIBRARIES}
    # m3api
    flycapture
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    apriltag
    apriltags_mit
    apriltags_swathmore
    CGAL
    yaml-cpp)

ADD_EXECUTABLE(atl_bridge_camera_node src/bridge/camera_node.cpp)
TARGET_LINK_LIBRARIES(atl_bridge_camera_node ${ATL_GAZEBO_DEPS})
ADD_DEPENDENCIES(atl_bridge_camera_node atl_gazebo_clients)

ADD_EXECUTABLE(atl_bridge_df_camera_node src/bridge/df_camera_node.cpp)
TARGET_LINK_LIBRARIES(atl_bridge_df_camera_node ${ATL_GAZEBO_DEPS})
ADD_DEPENDENCIES(atl_bridge_df_camera_node atl_gazebo_clients)

ADD_EXECUTABLE(atl_bridge_dji_quadrotor_node src/bridge/dji_quadrotor_node.cpp)
TARGET_LINK_LIBRARIES(atl_bridge_dji_quadrotor_node ${ATL_GAZEBO_DEPS})
ADD_DEPENDENCIES(atl_bridge_dji_quadrotor_node atl_gazebo_clients)

ADD_EXECUTABLE(atl_bridge_gimbal_node src/bridge/gimbal_node.cpp)
TARGET_LINK_LIBRARIES(atl_bridge_gimbal_node ${ATL_GAZEBO_DEPS})
ADD_DEPENDENCIES(atl_bridge_gimbal_node atl_gazebo_clients)

ADD_EXECUTABLE(atl_bridge_lz_node src/bridge/lz_node.cpp)
TARGET_LINK_LIBRARIES(atl_bridge_lz_node ${ATL_GAZEBO_DEPS})
ADD_DEPENDENCIES(atl_bridge_lz_node atl_gazebo_clients)

# ADD_EXECUTABLE(atl_bridge_px4_quadrotor_node src/bridge/px4_quadrotor_node.cpp)
# TARGET_LINK_LIBRARIES(atl_bridge_px4_quadrotor_node ${ATL_GAZEBO_DEPS})
# ADD_DEPENDENCIES(atl_bridge_px4_quadrotor_node atl_gazebo_clients)

ADD_EXECUTABLE(atl_bridge_world_node src/bridge/world_node.cpp)
TARGET_LINK_LIBRARIES(atl_bridge_world_node ${ATL_GAZEBO_DEPS})
ADD_DEPENDENCIES(atl_bridge_world_node atl_gazebo_clients)


# CLIENTS
SET(ATL_GAZEBO_CLIENTS_SRCS
    src/clients/camera_gclient.cpp
    src/clients/df_camera_gclient.cpp
    src/clients/gimbal_gclient.cpp
    src/clients/lz_gclient.cpp
    src/clients/quadrotor_gclient.cpp
    src/clients/world_gclient.cpp)
ADD_LIBRARY(atl_gazebo_clients ${ATL_GAZEBO_CLIENTS_SRCS})
ADD_DEPENDENCIES(atl_gazebo_clients atl_gazebo_msgs)


# CONTROLS
SET(ATL_GAZEBO_CONTROLS_SRCS src/controls/pid.cpp)
ADD_LIBRARY(atl_gazebo_controls ${ATL_GAZEBO_CONTROLS_SRCS})
ADD_DEPENDENCIES(atl_gazebo_controls atl_gazebo_msgs)


# KINEMATICS
SET(ATL_GAZEBO_KINEMATICS_SRCS
    src/kinematics/gimbal.cpp
    src/kinematics/quadrotor.cpp
    src/kinematics/two_wheel.cpp)
ADD_LIBRARY(atl_gazebo_kinematics ${ATL_GAZEBO_KINEMATICS_SRCS})
ADD_DEPENDENCIES(atl_gazebo_kinematics atl_gazebo_msgs)


# PLUGINS
SET(ATL_GAZEBO_PLUGINS_DEPS
		atl_core
    atl_gazebo_msgs
    atl_gazebo_kinematics
    atl_gazebo_controls
    ${GAZEBO_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp
    pthread)

ADD_LIBRARY(box_gplugin SHARED src/plugins/box_gplugin.cpp)
TARGET_LINK_LIBRARIES(box_gplugin ${ATL_GAZEBO_PLUGINS_DEPS})
ADD_DEPENDENCIES(box_gplugin atl_gazebo_msgs)

ADD_LIBRARY(camera_gplugin SHARED src/plugins/camera_gplugin.cpp)
TARGET_LINK_LIBRARIES(camera_gplugin ${ATL_GAZEBO_PLUGINS_DEPS})
ADD_DEPENDENCIES(camera_gplugin atl_gazebo_msgs)

ADD_LIBRARY(df_camera_gplugin SHARED src/plugins/df_camera_gplugin.cpp)
TARGET_LINK_LIBRARIES(df_camera_gplugin ${ATL_GAZEBO_PLUGINS_DEPS})
ADD_DEPENDENCIES(df_camera_gplugin atl_gazebo_msgs)

ADD_LIBRARY(gimbal_gplugin SHARED src/plugins/gimbal_gplugin.cpp)
TARGET_LINK_LIBRARIES(gimbal_gplugin ${ATL_GAZEBO_PLUGINS_DEPS})
ADD_DEPENDENCIES(gimbal_gplugin atl_gazebo_msgs)

ADD_LIBRARY(lz_gplugin SHARED src/plugins/lz_gplugin.cpp)
TARGET_LINK_LIBRARIES(lz_gplugin ${ATL_GAZEBO_PLUGINS_DEPS})
ADD_DEPENDENCIES(lz_gplugin atl_gazebo_msgs)

ADD_LIBRARY(quadrotor_gplugin SHARED src/plugins/quadrotor_gplugin.cpp)
TARGET_LINK_LIBRARIES(quadrotor_gplugin ${ATL_GAZEBO_PLUGINS_DEPS})
ADD_DEPENDENCIES(quadrotor_gplugin atl_gazebo_msgs)

ADD_LIBRARY(world_gplugin SHARED src/plugins/world_gplugin.cpp)
TARGET_LINK_LIBRARIES(world_gplugin ${ATL_GAZEBO_PLUGINS_DEPS})
ADD_DEPENDENCIES(world_gplugin atl_gazebo_msgs)


# ATL GAZEBO LIBRARY
ADD_LIBRARY(atl_gazebo
            STATIC
            ${ATL_GAZEBO_CLIENTS_SRCS}
            ${ATL_GAZEBO_CONTROLS_SRCS}
            ${ATL_GAZEBO_KINEMATICS_SRCS}
            ${PROTO_SRCS})
ADD_DEPENDENCIES(atl_gazebo atl_gazebo_clients atl_gazebo_msgs)
TARGET_LINK_LIBRARIES(atl_gazebo ${PROTOBUF_LIBRARY})


# INSTALL
INSTALL(DIRECTORY models
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
INSTALL(DIRECTORY worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
INSTALL(TARGETS
        box_gplugin
        camera_gplugin
        df_camera_gplugin
        gimbal_gplugin
        lz_gplugin
        quadrotor_gplugin
        world_gplugin
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins)
INSTALL(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
INSTALL(TARGETS atl_gazebo
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
