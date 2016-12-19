CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(awesomo)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

# DEPENDENCIES
FIND_PACKAGE(
    catkin
    REQUIRED COMPONENTS
    roscpp
    rospy
)

# INCLUDES
INCLUDE_DIRECTORIES(
    ${catkin_INCLUDE_DIRS}
    ./include/
    /usr/include/eigen3
)

# LIBRARY
ADD_LIBRARY(
    awesomo
    STATIC
    src/util.cpp
    src/controller.cpp
    src/quadrotor.cpp
    src/estimator.cpp
    src/estimator.cpp
    src/planner.cpp
)
SET(AWESOMO_DEPS
    awesomo
    yaml-cpp
    ${catkin_LIBRARIES}
)
INSTALL(
    TARGETS awesomo
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
INSTALL(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# UNIT TESTS
SET(TEST_DEST ${CATKIN_PACKAGE_BIN_DESTINATION})

ADD_EXECUTABLE(awesomo_controller_test tests/controller_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_controller_test ${AWESOMO_DEPS})

ADD_EXECUTABLE(awesomo_quadrotor_test tests/quadrotor_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_quadrotor_test ${AWESOMO_DEPS})

ADD_EXECUTABLE(awesomo_estimator_test tests/estimator_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_estimator_test ${AWESOMO_DEPS})

ADD_EXECUTABLE(awesomo_planner_test tests/planner_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_planner_test ${AWESOMO_DEPS})

ADD_EXECUTABLE(awesomo_util_test tests/util_test.cpp)
TARGET_LINK_LIBRARIES(awesomo_util_test ${AWESOMO_DEPS})

INSTALL(TARGETS awesomo_controller_test RUNTIME DESTINATION ${TEST_DEST})
INSTALL(TARGETS awesomo_quadrotor_test RUNTIME DESTINATION ${TEST_DEST})
INSTALL(TARGETS awesomo_estimator_test RUNTIME DESTINATION ${TEST_DEST})
INSTALL(TARGETS awesomo_quadrotor_test RUNTIME DESTINATION ${TEST_DEST})
INSTALL(TARGETS awesomo_util_test RUNTIME DESTINATION ${TEST_DEST})

# BUILD AS CATKIN PACKAGE
CATKIN_PACKAGE(INCLUDE_DIRS include LIBRARIES awesomo)
