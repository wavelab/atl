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
        message_runtime
        awesomo_core
        awesomo_msgs
)

# AWESOMO ROS UTILS
ADD_LIBRARY(
    awesomo_ros_utils
    src/ros/utils/msgs.cpp
    src/ros/utils/node.cpp
)
ADD_DEPENDENCIES(awesomo_ros_utils awesomo_msgs_gencpp)
SET(
    DEPS
    awesomo_ros_utils
    awesomo_core
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    apriltags_swathmore
    apriltags_mit
    CGAL
    yaml-cpp
)

# ROS NODES
ADD_EXECUTABLE(apriltag_node src/ros/nodes/apriltag_node.cpp)
TARGET_LINK_LIBRARIES(apriltag_node ${DEPS})

ADD_EXECUTABLE(camera_node src/ros/nodes/camera_node.cpp)
TARGET_LINK_LIBRARIES(camera_node ${DEPS})

ADD_EXECUTABLE(control_node src/ros/nodes/control_node.cpp)
TARGET_LINK_LIBRARIES(control_node ${DEPS})

ADD_EXECUTABLE(estimate_node src/ros/nodes/estimate_node.cpp)
TARGET_LINK_LIBRARIES(estimate_node ${DEPS})

ADD_EXECUTABLE(example_node src/ros/nodes/example_node.cpp)
TARGET_LINK_LIBRARIES(example_node ${DEPS})

ADD_EXECUTABLE(gimbal_node src/ros/nodes/gimbal_node.cpp)
TARGET_LINK_LIBRARIES(gimbal_node ${DEPS})

# ROS NODE TESTS
SET(TEST_DEPS ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
FILE(
    COPY tests/test_data
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
)

ADD_ROSTEST_GTEST(
    apriltag_test
    tests/launch/apriltag_test.launch
    tests/apriltag_test.cpp
)
target_link_libraries(apriltag_test ${TEST_DEPS})

ADD_ROSTEST_GTEST(
    camera_test
    tests/launch/camera_test.launch
    tests/camera_test.cpp
)
target_link_libraries(camera_test ${TEST_DEPS})

# INSTALL LAUNCH FILES
SET(SHARE_DEST ${CATKIN_PACKAGE_SHARE_DESTINATION})
INSTALL(DIRECTORY launch/ DESTINATION ${SHARE_DEST}/launch)
