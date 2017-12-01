# TEST SETTINGS
GET_FILENAME_COMPONENT(TEST_DATA_DIR "data" ABSOLUTE)
GET_FILENAME_COMPONENT(TEST_WORLD_DIR "worlds" ABSOLUTE)
CONFIGURE_FILE("test_settings.hpp.in" "test_settings.hpp")
INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR}/tests)

# TESTS
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/tests/)
FILE(COPY data DESTINATION ${PROJECT_BINARY_DIR}/tests/)
FILE(COPY worlds DESTINATION ${PROJECT_BINARY_DIR}/tests/)
SET(
    TEST_DEPS
    wave
    atl_gazebo_clients
    atl_gazebo_msgs
    ${GAZEBO_LIBRARIES}
    gtest
    pthread
    ${OpenCV_LIBS}
    yaml-cpp
)

# ADD_EXECUTABLE(gazebo_camera_test camera_test.cpp)
# TARGET_LINK_LIBRARIES(gazebo_camera_test camera_gplugin ${TEST_DEPS})

# ADD_EXECUTABLE(gazebo_gimbal_test gimbal_test.cpp)
# TARGET_LINK_LIBRARIES(gazebo_gimbal_test gimbal_gplugin ${TEST_DEPS})

# ADD_EXECUTABLE(gazebo_quadrotor_test quadrotor_test.cpp)
# TARGET_LINK_LIBRARIES(gazebo_quadrotor_test ${TEST_DEPS} quadrotor_gplugin)

# ADD_EXECUTABLE(gazebo_world_test world_test.cpp)
# TARGET_LINK_LIBRARIES(gazebo_world_test world_gplugin ${TEST_DEPS})
