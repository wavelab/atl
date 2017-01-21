# ROS NODE TESTS
SET(TEST_DEPS ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
FILE(
    COPY nodes/test_data
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
)

ADD_ROSTEST_GTEST(
    apriltag_test
    launch/apriltag_test.launch
    nodes/apriltag_test.cpp
)
TARGET_LINK_LIBRARIES(apriltag_test ${TEST_DEPS})

ADD_ROSTEST_GTEST(
    camera_test
    launch/camera_test.launch
    nodes/camera_test.cpp
)
TARGET_LINK_LIBRARIES(camera_test ${TEST_DEPS})

ADD_ROSTEST_GTEST(
    estimator_test
    launch/estimator_test.launch
    nodes/estimator_test.cpp
)
TARGET_LINK_LIBRARIES(estimator_test ${TEST_DEPS})

ADD_ROSTEST_GTEST(
    gimbal_test
    launch/gimbal_test.launch
    nodes/gimbal_test.cpp
)
TARGET_LINK_LIBRARIES(gimbal_test ${TEST_DEPS})
