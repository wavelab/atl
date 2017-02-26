SET(
    DEPS
    awesomo_ros_utils
    awesomo_core
    flycapture
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    apriltag
    apriltags_mit
    apriltags_swathmore
    CGAL
    yaml-cpp
    m3api
)

# ROS NODES
ADD_EXECUTABLE(apriltag_node ros/nodes/apriltag_node.cpp)
TARGET_LINK_LIBRARIES(apriltag_node ${DEPS})

ADD_EXECUTABLE(camera_node ros/nodes/camera_node.cpp)
TARGET_LINK_LIBRARIES(camera_node ${DEPS})

ADD_EXECUTABLE(control_node ros/nodes/control_node.cpp)
TARGET_LINK_LIBRARIES(control_node ${DEPS})

ADD_EXECUTABLE(estimator_node ros/nodes/estimator_node.cpp)
TARGET_LINK_LIBRARIES(estimator_node ${DEPS})

ADD_EXECUTABLE(example_node ros/nodes/example_node.cpp)
TARGET_LINK_LIBRARIES(example_node ${DEPS})

ADD_EXECUTABLE(fake_mavros_node ros/nodes/fake_mavros_node.cpp)
TARGET_LINK_LIBRARIES(fake_mavros_node ${DEPS})

ADD_EXECUTABLE(gimbal_node ros/nodes/gimbal_node.cpp)
TARGET_LINK_LIBRARIES(gimbal_node ${DEPS})

ADD_EXECUTABLE(imu_node ros/nodes/imu_node.cpp)
TARGET_LINK_LIBRARIES(imu_node ${DEPS})

ADD_EXECUTABLE(pg_camera_node ros/nodes/pg_camera_node.cpp)
TARGET_LINK_LIBRARIES(pg_camera_node ${DEPS})

ADD_EXECUTABLE(ximea_camera_node ros/nodes/ximea_camera_node.cpp)
TARGET_LINK_LIBRARIES(ximea_camera_node ${DEPS})

# ROS UTILS
ADD_LIBRARY(
    awesomo_ros_utils
    ros/utils/msgs.cpp
    ros/utils/node.cpp
)
ADD_DEPENDENCIES(awesomo_ros_utils awesomo_msgs_gencpp)
