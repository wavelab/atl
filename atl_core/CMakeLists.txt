CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(atl_core)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC")
IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

# CATKIN
FIND_PACKAGE(catkin)
FIND_PACKAGE(OpenCV 3.0 REQUIRED)
FIND_PACKAGE(CGAL)

# INCLUDES
INCLUDE_DIRECTORIES(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    include
    /usr/include/eigen3
    /usr/include/flycapture/
    /usr/include/m3api/
)

# LIBS
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})
LINK_DIRECTORIES(${catkin_LIBRARY_DIRS})


CATKIN_PACKAGE(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

# LIBRARY
ADD_LIBRARY(
    ${PROJECT_NAME}
    STATIC
    # control
    src/control/landing_controller.cpp
    src/control/pid.cpp
    src/control/position_controller.cpp
    src/control/tracking_controller.cpp
    src/control/trajectory.cpp
    src/control/trajectory_controller.cpp
    src/control/trajectory_index.cpp
    src/control/velocity_controller.cpp
    src/control/waypoint_controller.cpp
    # data
    src/data/attitude_command.cpp
    src/data/landing_target.cpp
    src/data/transform.cpp
    src/data/pose.cpp
    # estimation
    src/estimation/ekf.cpp
    src/estimation/ekf_tracker.cpp
    src/estimation/kf.cpp
    src/estimation/kf_tracker.cpp
    # mission
    src/mission/mission.cpp
    # models
    src/models/gimbal.cpp
    src/models/quadrotor.cpp
    src/models/two_wheel.cpp
    # planning
    src/planning/model.cpp
    src/planning/optimizer.cpp
    src/planning/trajectory.cpp
    src/planning/utils.cpp
    # quadrotor
    src/quadrotor/quadrotor.cpp
    # sensor
    src/sensor/i2c.cpp
    src/sensor/MPU6050.cpp
    # utils
    src/utils/config.cpp
    src/utils/data.cpp
    src/utils/file.cpp
    src/utils/gps.cpp
    src/utils/math.cpp
    src/utils/opencv.cpp
    src/utils/stats.cpp
    src/utils/time.cpp
    # vision
    src/vision/apriltag/base_detector.cpp
    src/vision/apriltag/michigan.cpp
    src/vision/apriltag/mit.cpp
    src/vision/apriltag/swathmore.cpp
    src/vision/camera/camera.cpp
    src/vision/camera/config.cpp
    src/vision/camera/dc1394.cpp
    src/vision/camera/pointgrey.cpp
    src/vision/camera/ximea.cpp
    src/vision/gimbal/gimbal.cpp
    src/vision/gimbal/sbgc.cpp
)
SET(${PROJECT_NAME}_DEPS
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    m3api
    flycapture
    dc1394
    apriltag
    apriltags_mit
    apriltags_swathmore
    CGAL
    nlopt
    yaml-cpp
    gtest
    pthread
)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})

# UNIT TESTS
ADD_EXECUTABLE(
    atl_tests
    # control
    tests/control/landing_controller_test.cpp
    tests/control/pid_test.cpp
    tests/control/position_controller_test.cpp
    tests/control/tracking_controller_test.cpp
    tests/control/trajectory_index_test.cpp
    tests/control/trajectory_test.cpp
    tests/control/velocity_controller_test.cpp
    tests/control/waypoint_controller_test.cpp
    # data
    tests/data/landing_target_test.cpp
    tests/data/pose_test.cpp
    tests/data/transform_test.cpp
    # estimation
    tests/estimation/ekf_test.cpp
    tests/estimation/ekf_tracker_test.cpp
    tests/estimation/kf_test.cpp
    tests/estimation/kf_tracker_test.cpp
    # mission
    tests/mission/mission_test.cpp
    # planning
    tests/planning/model_test.cpp
    tests/planning/optimizer_test.cpp
    tests/planning/trajectory_test.cpp
    tests/planning/utils_test.cpp
    # quadrotor
    tests/quadrotor/quadrotor_test.cpp
    # sensor
    tests/sensor/MPU6050_test.cpp
    # vision
    tests/vision/apriltag/mit_test.cpp
    tests/vision/camera/camera_test.cpp
    tests/vision/camera/config_test.cpp
    tests/vision/camera/dc1394_test.cpp
    tests/vision/camera/pointgrey_test.cpp
    tests/vision/gimbal/gimbal_test.cpp
    tests/vision/gimbal/sbgc_test.cpp
    # util
    tests/utils/config_test.cpp
    tests/utils/data_test.cpp
    tests/utils/file_test.cpp
    tests/utils/gps_test.cpp
    tests/utils/math_test.cpp
    tests/utils/opencv_test.cpp
    tests/utils/stats_test.cpp
    tests/utils/time_test.cpp
    # test runner
    tests/test_runner.cpp
)
TARGET_LINK_LIBRARIES(atl_tests ${PROJECT_NAME} ${${PROJECT_NAME}_DEPS})

# INSTALL
INSTALL(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
INSTALL(
    DIRECTORY include/atl/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
