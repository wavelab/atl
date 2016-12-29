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
    ${catkin_LIBRARY_DIRS}
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
    CATKIN_DEPENDS roscpp rospy std_msgs message_runtime awesomo_core
)

SET(DEPS
    awesomo_core
    ${catkin_LIBRARIES}
    yaml-cpp
)

# ROS NODES
SET(BIN_DEST ${CATKIN_PACKAGE_BIN_DESTINATION})

ADD_EXECUTABLE(camera_node src/ros/camera_node.cpp)
ADD_DEPENDENCIES(camera_node ${PROJECT_NAME}_gencpp awesomo_core)
TARGET_LINK_LIBRARIES(camera_node ${DEPS})
INSTALL(TARGETS camera_node RUNTIME DESTINATION ${BIN_DEST})

ADD_EXECUTABLE(control_node src/ros/control_node.cpp)
ADD_DEPENDENCIES(control_node ${PROJECT_NAME}_gencpp awesomo_core)
TARGET_LINK_LIBRARIES(control_node ${DEPS})
INSTALL(TARGETS control_node RUNTIME DESTINATION ${BIN_DEST})

ADD_EXECUTABLE(example_node src/ros/example_node.cpp)
ADD_DEPENDENCIES(example_node ${PROJECT_NAME}_gencpp awesomo_core)
TARGET_LINK_LIBRARIES(example_node ${DEPS})
INSTALL(TARGETS example_node RUNTIME DESTINATION ${BIN_DEST})

ADD_EXECUTABLE(gimbal_node src/ros/gimbal_node.cpp)
ADD_DEPENDENCIES(gimbal_node ${PROJECT_NAME}_gencpp awesomo_core)
TARGET_LINK_LIBRARIES(gimbal_node ${DEPS})
INSTALL(TARGETS gimbal_node RUNTIME DESTINATION ${BIN_DEST})

# INSTALL LAUNCH FILES
SET(SHARE_DEST ${CATKIN_PACKAGE_SHARE_DESTINATION})
INSTALL(DIRECTORY launch/ DESTINATION ${SHARE_DEST}/launch)
