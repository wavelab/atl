CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(atl_ros)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_PACKAGE_LIB_DESTINATION})

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
    dji_sdk
    atl_core
    atl_msgs
)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Boost REQUIRED)

# INCLUDES
INCLUDE_DIRECTORIES(include)
INCLUDE_DIRECTORIES(/usr/include)
INCLUDE_DIRECTORIES(/usr/local/include)
INCLUDE_DIRECTORIES(/usr/include/eigen3)
INCLUDE_DIRECTORIES(/usr/include/opencv2)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES(SYSTEM ${catkin_INCLUDE_DIRS})

# LIBS
LINK_DIRECTORIES(/usr/lib)
LINK_DIRECTORIES(/usr/local/lib)
LINK_DIRECTORIES(AFTER ${catkin_LIBRARY_DIRS})

# BUILD AS CATKIN PACKAGE
CATKIN_PACKAGE(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_runtime
    atl_core
    atl_msgs
  LIBRARIES
    atl_ros_utils
)

# ROS NODES
SET(
    ATL_ROS_DEPS
    atl_ros_utils
    atl_core
    m3api
    flycapture
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    apriltag
    apriltags_mit
    apriltags_swathmore
    CGAL
    yaml-cpp
)
ADD_EXECUTABLE(atl_apriltag_node src/ros/nodes/apriltag_node.cpp)
TARGET_LINK_LIBRARIES(atl_apriltag_node ${ATL_ROS_DEPS})
ADD_DEPENDENCIES(atl_apriltag_node atl_ros_utils)

ADD_EXECUTABLE(atl_camera_node src/ros/nodes/camera_node.cpp)
TARGET_LINK_LIBRARIES(atl_camera_node ${ATL_ROS_DEPS})
ADD_DEPENDENCIES(atl_camera_node atl_ros_utils)

ADD_EXECUTABLE(atl_control_node src/ros/nodes/control_node.cpp)
TARGET_LINK_LIBRARIES(atl_control_node ${ATL_ROS_DEPS})
ADD_DEPENDENCIES(atl_control_node atl_ros_utils)

ADD_EXECUTABLE(atl_estimator_node src/ros/nodes/estimator_node.cpp)
TARGET_LINK_LIBRARIES(atl_estimator_node ${ATL_ROS_DEPS})
ADD_DEPENDENCIES(atl_estimator_node atl_ros_utils)

ADD_EXECUTABLE(atl_example_node src/ros/nodes/example_node.cpp)
TARGET_LINK_LIBRARIES(atl_example_node ${ATL_ROS_DEPS})
ADD_DEPENDENCIES(atl_example_node atl_ros_utils)

ADD_EXECUTABLE(atl_fake_mavros_node src/ros/nodes/fake_mavros_node.cpp)
TARGET_LINK_LIBRARIES(atl_fake_mavros_node ${ATL_ROS_DEPS})
ADD_DEPENDENCIES(atl_fake_mavros_node atl_ros_utils)

ADD_EXECUTABLE(atl_gimbal_node src/ros/nodes/gimbal_node.cpp)
TARGET_LINK_LIBRARIES(atl_gimbal_node ${ATL_ROS_DEPS})
ADD_DEPENDENCIES(atl_gimbal_node atl_ros_utils)

ADD_EXECUTABLE(atl_imu_node src/ros/nodes/imu_node.cpp)
TARGET_LINK_LIBRARIES(atl_imu_node ${ATL_ROS_DEPS})
ADD_DEPENDENCIES(atl_imu_node atl_ros_utils)

# ADD_EXECUTABLE(atl_pg_camera_node src/ros/nodes/pg_camera_node.cpp)
# TARGET_LINK_LIBRARIES(atl_pg_camera_node ${ATL_ROS_DEPS})
# ADD_DEPENDENCIES(atl_pg_camera_node atl_ros_utils)
#
# ADD_EXECUTABLE(atl_ximea_camera_node src/ros/nodes/ximea_camera_node.cpp)
# TARGET_LINK_LIBRARIES(atl_ximea_camera_node ${ATL_ROS_DEPS})
# ADD_DEPENDENCIES(atl_ximea_camera_node atl_ros_utils)

# ROS UTILS
ADD_LIBRARY(
    atl_ros_utils
    src/ros/utils/msgs.cpp
    src/ros/utils/node.cpp
    src/ros/utils/utils.cpp
)
# ADD_DEPENDENCIES(atl_ros_utils atl_msgs_gencpp)


# TESTS
# SET(ATL_ROS_TEST_DEPS ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
# FILE(
#     COPY tests/nodes/test_data
#     DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
# )
#
# ADD_ROSTEST_GTEST(
#     apriltag_test
#     tests/launch/apriltag_test.launch
#     tests/nodes/apriltag_test.cpp
# )
# TARGET_LINK_LIBRARIES(apriltag_test ${ATL_ROS_TEST_DEPS})
#
# ADD_ROSTEST_GTEST(
#     camera_test
#     tests/launch/camera_test.launch
#     tests/nodes/camera_test.cpp
# )
# TARGET_LINK_LIBRARIES(camera_test ${ATL_ROS_TEST_DEPS})
#
# ADD_ROSTEST_GTEST(
#     estimator_test
#     tests/launch/estimator_test.launch
#     tests/nodes/estimator_test.cpp
# )
# TARGET_LINK_LIBRARIES(estimator_test ${ATL_ROS_TEST_DEPS})
#
# ADD_ROSTEST_GTEST(
#     gimbal_test
#     tests/launch/gimbal_test.launch
#     tests/nodes/gimbal_test.cpp
# )
# TARGET_LINK_LIBRARIES(gimbal_test ${ATL_ROS_TEST_DEPS})

# INSTALL LAUNCH FILES
SET(SHARE_DEST ${CATKIN_PACKAGE_SHARE_DESTINATION})
INSTALL(DIRECTORY launch/ DESTINATION ${SHARE_DEST}/launch)
