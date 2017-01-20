SET(
    DEPS
    awesomo_ros_utils
    awesomo_core
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    # apriltags_swathmore
    apriltags_mit
    CGAL
    yaml-cpp
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

ADD_EXECUTABLE(gimbal_node ros/nodes/gimbal_node.cpp)
TARGET_LINK_LIBRARIES(gimbal_node ${DEPS})

# ROS UTILS
ADD_LIBRARY(
    awesomo_ros_utils
    ros/utils/msgs.cpp
    ros/utils/node.cpp
)
ADD_DEPENDENCIES(awesomo_ros_utils awesomo_msgs_gencpp)
