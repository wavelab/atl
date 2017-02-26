CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(awesomo_core)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

# CATKIN
FIND_PACKAGE(catkin)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(CGAL)
CATKIN_PACKAGE(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

# INCLUDES
INCLUDE_DIRECTORIES(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    include
    /usr/include
    /usr/local/include
    /usr/include/eigen3
    /usr/include/flycapture/
    /usr/include/m3api/
)

# LIBRARY
ADD_LIBRARY(
    ${PROJECT_NAME}
    STATIC
    # control
    src/control/landing_controller.cpp
    src/control/pid_controller.cpp
    src/control/position_controller.cpp
    src/control/tracking_controller.cpp
    src/control/velocity_controller.cpp
    # estimation
    src/estimation/ekf.cpp
    src/estimation/ekf_tracker.cpp
    src/estimation/kf.cpp
    src/estimation/kf_tracker.cpp
    # planning
    src/planning/ga.cpp
    src/planning/model.cpp
    src/planning/optimizer.cpp
    src/planning/trajectory.cpp
    src/planning/utils.cpp
    # quadrotor
    src/quadrotor/landing_target.cpp
    src/quadrotor/quadrotor.cpp
    # sensor
    src/sensor/i2c.cpp
    src/sensor/MPU6050.cpp
    # utils
    src/utils/config.cpp
    src/utils/data.cpp
    src/utils/filesystem.cpp
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
    src/vision/camera/ximea.cpp
    src/vision/camera/pointgrey.cpp
    src/vision/gimbal/gimbal.cpp
    src/vision/gimbal/sbgc.cpp
)
SET(DEPS
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    m3api
    flycapture
    apriltag
    apriltags_mit
    apriltags_swathmore
    CGAL
    nlopt
    yaml-cpp
    gtest
    pthread
)

# UNIT TESTS
ADD_EXECUTABLE(
    awesomo_tests
    # CONTROL
    tests/control/landing_controller_test.cpp
    tests/control/pid_controller_test.cpp
    tests/control/position_controller_test.cpp
    tests/control/tracking_controller_test.cpp
    tests/control/velocity_controller_test.cpp
    # ESTIMATION
    tests/estimation/ekf_test.cpp
    tests/estimation/ekf_tracker_test.cpp
    tests/estimation/kf_test.cpp
    tests/estimation/kf_tracker_test.cpp
    # PLANNING
    tests/planning/ga_test.cpp
    tests/planning/model_test.cpp
    tests/planning/optimizer_test.cpp
    tests/planning/trajectory_test.cpp
    tests/planning/utils_test.cpp
    # QUADROTOR
    tests/quadrotor/landing_target_test.cpp
    tests/quadrotor/quadrotor_test.cpp
    # SENSOR
    tests/sensor/MPU6050_test.cpp
    # VISION
    tests/vision/apriltag/mit_test.cpp
    tests/vision/camera/camera_test.cpp
    tests/vision/camera/pointgrey_test.cpp
    tests/vision/camera/config_test.cpp
    tests/vision/gimbal/gimbal_test.cpp
    tests/vision/gimbal/sbgc_test.cpp
    # UTIL
    tests/utils/config_test.cpp
    tests/utils/data_test.cpp
    tests/utils/filesystem_test.cpp
    tests/utils/math_test.cpp
    tests/utils/opencv_test.cpp
    tests/utils/stats_test.cpp
    tests/utils/time_test.cpp
    # TEST RUNNER
    tests/test_runner.cpp
)
TARGET_LINK_LIBRARIES(awesomo_tests ${DEPS})

# INSTALL
INSTALL(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
INSTALL(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
