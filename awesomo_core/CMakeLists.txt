CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(awesomo_core)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

# CATKIN
FIND_PACKAGE(catkin)
CATKIN_PACKAGE(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})
FIND_PACKAGE(OpenCV REQUIRED)

# INCLUDES
INCLUDE_DIRECTORIES(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    include
    /usr/include
    /usr/local/include
    /usr/include/eigen3
)

# LIBRARY
ADD_LIBRARY(
    ${PROJECT_NAME}
    STATIC
    # control
    src/control/attitude_controller.cpp
    src/control/pid_controller.cpp
    src/control/position_controller.cpp
    src/control/velocity_controller.cpp
    # estimation
    src/estimation/ekf.cpp
    src/estimation/kf.cpp
    # planning
    src/planning/planning.cpp
    # quadrotor
    src/quadrotor/base_mode.cpp
    src/quadrotor/hover_mode.cpp
    src/quadrotor/landing_mode.cpp
    src/quadrotor/quadrotor.cpp
    # utils
    src/utils/config.cpp
    src/utils/data.cpp
    src/utils/io.cpp
    src/utils/math.cpp
    src/utils/opencv.cpp
    src/utils/stats.cpp
    src/utils/time.cpp
    # vision
    src/vision/camera/camera.cpp
    src/vision/camera/camera_config.cpp
    src/vision/camera/camera_ximea.cpp
    # src/vision/camera/camera_pointgrey.cpp
)
SET(DEPS
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp
    gtest
    pthread
)

# UNIT TESTS
# CONTROL TESTS
ADD_EXECUTABLE(awesomo_attitude_controller_test tests/control/attitude_controller_test.cpp)
ADD_EXECUTABLE(awesomo_pid_controller_test tests/control/pid_controller_test.cpp)
ADD_EXECUTABLE(awesomo_position_controller_test tests/control/position_controller_test.cpp)
ADD_EXECUTABLE(awesomo_velocity_controller_test tests/control/velocity_controller_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_attitude_controller_test ${DEPS})
TARGET_LINK_LIBRARIES(awesomo_pid_controller_test ${DEPS})
TARGET_LINK_LIBRARIES(awesomo_position_controller_test ${DEPS})
TARGET_LINK_LIBRARIES(awesomo_velocity_controller_test ${DEPS})

# ESTIMATION TESTS
ADD_EXECUTABLE(awesomo_ekf_test tests/estimation/ekf_test.cpp)
ADD_EXECUTABLE(awesomo_kf_test tests/estimation/kf_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_ekf_test ${DEPS})
TARGET_LINK_LIBRARIES(awesomo_kf_test ${DEPS})

# PLANNING TESTS
ADD_EXECUTABLE(awesomo_planning_test tests/planning/planning_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_planning_test ${DEPS})

# QUADROTOR TESTS
ADD_EXECUTABLE(awesomo_hover_mode_test tests/quadrotor/hover_mode_test.cpp)
ADD_EXECUTABLE(awesomo_quadrotor_test tests/quadrotor/quadrotor_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_hover_mode_test ${DEPS})
TARGET_LINK_LIBRARIES(awesomo_quadrotor_test ${DEPS})

# VISION TESTS
ADD_EXECUTABLE(awesomo_camera_test tests/vision/camera_test.cpp)
ADD_EXECUTABLE(awesomo_camera_config_test tests/vision/camera_config_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_camera_test ${DEPS})
TARGET_LINK_LIBRARIES(awesomo_camera_config_test ${DEPS})

# UTIL TESTS
ADD_EXECUTABLE(awesomo_config_test tests/utils/config_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_config_test ${DEPS})

# TESTS
ADD_EXECUTABLE(awesomo_test tests/test_runner.cpp)
TARGET_LINK_LIBRARIES(awesomo_test ${DEPS})

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

# SET(TEST_DEST ${CATKIN_PACKAGE_BIN_DESTINATION})
# INSTALL(DIRECTORY tests/configs DESTINATION ${TEST_DEST})
# INSTALL(TARGETS awesomo_controller_test RUNTIME DESTINATION ${TEST_DEST})
# INSTALL(TARGETS awesomo_quadrotor_test RUNTIME DESTINATION ${TEST_DEST})
# INSTALL(TARGETS awesomo_estimator_test RUNTIME DESTINATION ${TEST_DEST})
# INSTALL(TARGETS awesomo_quadrotor_test RUNTIME DESTINATION ${TEST_DEST})
# INSTALL(TARGETS awesomo_util_test RUNTIME DESTINATION ${TEST_DEST})
