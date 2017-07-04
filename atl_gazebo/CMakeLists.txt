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
FIND_PACKAGE(catkin REQUIRED COMPONENTS atl_core)
CATKIN_PACKAGE(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

# INCLUDES
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(/usr/include/eigen3)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GAZEBO_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/msgs)

# LIBS
LINK_DIRECTORIES(/usr/lib)
LINK_DIRECTORIES(/usr/local/lib)
LINK_DIRECTORIES(${GAZEBO_LIBRARY_DIRS})
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})
LINK_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/gazebo)
LINK_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR}/lib)

# MSGS
SET(PROTOBUF_IMPORT_DIRS)
FOREACH(ITR ${GAZEBO_INCLUDE_DIRS})
    IF(ITR MATCHES ".*gazebo-[0-9.]+$")
        SET(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    ENDIF()
ENDFOREACH()
SET(
    GAZEBO_MSGS
    msgs/attitude_setpoint.proto
    msgs/model_pose.proto
    msgs/position_setpoint.proto
    msgs/rpy_pose.proto
    msgs/velocity_setpoint.proto
)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${GAZEBO_MSGS})
ADD_LIBRARY(atl_gazebo_msgs STATIC ${PROTO_SRCS})
TARGET_LINK_LIBRARIES(atl_gazebo_msgs ${PROTOBUF_LIBRARY})

# CLIENTS
SET(ATL_GAZEBO_CLIENTS_SRCS
    src/clients/camera_gclient.cpp
    src/clients/df_camera_gclient.cpp
    src/clients/gimbal_gclient.cpp
    src/clients/lz_gclient.cpp
    src/clients/quadrotor_gclient.cpp
    src/clients/world_gclient.cpp)
ADD_LIBRARY(atl_gazebo_clients STATIC ${ATL_GAZEBO_CLIENTS_SRCS})
ADD_DEPENDENCIES(atl_gazebo_clients atl_gazebo_msgs)

# CONTROLS
SET(ATL_GAZEBO_CONTROLS_SRCS src/controls/pid.cpp)
ADD_LIBRARY(atl_gazebo_controls STATIC ${ATL_GAZEBO_CONTROLS_SRCS})
ADD_DEPENDENCIES(atl_gazebo_controls atl_gazebo_msgs)

# KINEMATICS
SET(ATL_GAZEBO_KINEMATICS_SRCS
    src/kinematics/gimbal.cpp
    src/kinematics/quadrotor.cpp
    src/kinematics/two_wheel.cpp)
ADD_LIBRARY(atl_gazebo_kinematics STATIC ${ATL_GAZEBO_KINEMATICS_SRCS})
ADD_DEPENDENCIES(atl_gazebo_kinematics atl_gazebo_msgs)

# PLUGINS
SET(ATL_GAZEBO_PLUGINS_DEPS
    atl_gazebo_msgs
    atl_gazebo_controls
    atl_gazebo_kinematics
    wave
    ${GAZEBO_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp
    pthread)
ADD_LIBRARY(box_gplugin SHARED src/plugins/box_gplugin.cpp)
ADD_DEPENDENCIES(box_gplugin atl_gazebo_msgs)
TARGET_LINK_LIBRARIES(box_gplugin ${ATL_GAZEBO_PLUGINS_DEPS})

ADD_LIBRARY(camera_gplugin SHARED src/plugins/camera_gplugin.cpp)
ADD_DEPENDENCIES(camera_gplugin atl_gazebo_msgs)
TARGET_LINK_LIBRARIES(camera_gplugin ${ATL_GAZEBO_PLUGINS_DEPS})

ADD_LIBRARY(df_camera_gplugin SHARED src/plugins/df_camera_gplugin.cpp)
ADD_DEPENDENCIES(df_camera_gplugin atl_gazebo_msgs)
TARGET_LINK_LIBRARIES(df_camera_gplugin ${ATL_GAZEBO_PLUGINS_DEPS})

ADD_LIBRARY(gimbal_gplugin SHARED src/plugins/gimbal_gplugin.cpp)
ADD_DEPENDENCIES(gimbal_gplugin atl_gazebo_msgs)
TARGET_LINK_LIBRARIES(gimbal_gplugin ${ATL_GAZEBO_PLUGINS_DEPS})

ADD_LIBRARY(lz_gplugin SHARED src/plugins/lz_gplugin.cpp)
ADD_DEPENDENCIES(lz_gplugin atl_gazebo_msgs)
TARGET_LINK_LIBRARIES(lz_gplugin ${ATL_GAZEBO_PLUGINS_DEPS})

ADD_LIBRARY(quadrotor_gplugin SHARED src/plugins/quadrotor_gplugin.cpp)
ADD_DEPENDENCIES(quadrotor_gplugin atl_gazebo_msgs)
TARGET_LINK_LIBRARIES(quadrotor_gplugin ${ATL_GAZEBO_PLUGINS_DEPS})

ADD_LIBRARY(world_gplugin SHARED src/plugins/world_gplugin.cpp)
ADD_DEPENDENCIES(world_gplugin atl_gazebo_msgs)
TARGET_LINK_LIBRARIES(world_gplugin ${ATL_GAZEBO_PLUGINS_DEPS})

# atl GAZEBO LIBRARY
ADD_LIBRARY(atl_gazebo
            STATIC
            ${ATL_GAZEBO_CLIENTS_SRCS}
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
        # quadrotor_gplugin
        world_gplugin
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugins)
INSTALL(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
INSTALL(TARGETS atl_gazebo
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
