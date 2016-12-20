CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(awesomo_core)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

# CATKIN
FIND_PACKAGE(catkin)
CATKIN_PACKAGE(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME})

# INCLUDES
INCLUDE_DIRECTORIES(
    ${catkin_INCLUDE_DIRS}
    include
    /usr/include
    /usr/local/include
    /usr/include/eigen3
)

# LIBRARY
ADD_LIBRARY(
    ${PROJECT_NAME}
    STATIC
    src/control/attitude_controller.cpp
    src/control/pid_controller.cpp
    src/control/position_controller.cpp
    src/control/velocity_controller.cpp
    src/estimation/kf.cpp
    src/planning/planner.cpp
    src/utils/config.cpp
    src/utils/math.cpp
    src/utils/data.cpp
    src/utils/stats.cpp
    src/utils/time.cpp
    src/quadrotor.cpp
)
SET(DEPS
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    yaml-cpp
    gtest
    pthread
)

# UNIT TESTS
ADD_EXECUTABLE(awesomo_attitude_controller_test tests/control/attitude_controller_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_attitude_controller_test ${DEPS})

ADD_EXECUTABLE(awesomo_pid_controller_test tests/control/pid_controller_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_pid_controller_test ${DEPS})

ADD_EXECUTABLE(awesomo_position_controller_test tests/control/position_controller_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_position_controller_test ${DEPS})

ADD_EXECUTABLE(awesomo_velocity_controller_test tests/control/velocity_controller_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_velocity_controller_test ${DEPS})


# ADD_EXECUTABLE(awesomo_quadrotor_test tests/quadrotor_test.cpp)
# TARGET_LINK_LIBRARIES(awesomo_quadrotor_test ${DEPS})
#
# ADD_EXECUTABLE(awesomo_estimator_test tests/estimator_test.cpp)
# TARGET_LINK_LIBRARIES(awesomo_estimator_test ${DEPS})
#
# ADD_EXECUTABLE(awesomo_planner_test tests/planner_test.cpp)
# TARGET_LINK_LIBRARIES(awesomo_planner_test ${DEPS})
#
# ADD_EXECUTABLE(awesomo_util_test tests/util_test.cpp)
# TARGET_LINK_LIBRARIES(awesomo_util_test ${DEPS})

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
