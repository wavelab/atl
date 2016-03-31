CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
PROJECT(awesomo)

IF (CMAKE_COMPILER_IS_GNUCXX)
    ADD_DEFINITIONS(-std=gnu++0x)
ENDIF()

# OPENMP
FIND_PACKAGE(OpenMP REQUIRED)
if (OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# DEPENDENCIES
FIND_PACKAGE(
    catkin
    REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    image_transport
    cv_bridge
)
FIND_PACKAGE(OpenCV)
FIND_PACKAGE(Boost)

# LIBS
LINK_DIRECTORIES(
    /usr/local/lib
    /usr/lib
)

# INCLUDES
INCLUDE_DIRECTORIES(
    ${catkin_INCLUDE_DIRS}
    /usr/local/include
    /usr/include/eigen3
    /usr/include/flycapture
    $(Boost_INCLUDE_DIR)
    ./include
)

# BUILD AS CATKIN PACKAGE
CATKIN_PACKAGE(
    INCLUDE_DIRS include
    LIBRARIES awesomo
    CATKIN_DEPENDS roscpp rospy std_msgs
    DEPENDS system_lib
)

# LIBRARY
ADD_LIBRARY(
    awesomo
    STATIC
    src/sim.cpp
    src/pid.cpp
    src/util.cpp
    src/camera.cpp
    # src/quadrotor.cpp
    src/apriltag.cpp
)


## ROS NODE EXECUTABLES
# camera_node
ADD_EXECUTABLE(camera_node src/ros/camera_node.cpp)
target_link_libraries(
    camera_node
    awesomo
    apriltags
    yaml-cpp
    flycapture
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
INSTALL(
    TARGETS camera_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# quadrotor_node
ADD_EXECUTABLE(quadrotor_node src/ros/quadrotor_node.cpp)
TARGET_LINK_LIBRARIES(quadrotor_node ${catkin_LIBRARIES} awesomo)
INSTALL(
    TARGETS quadrotor_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## UNIT TESTS
# camera_test
ADD_EXECUTABLE(camera_test tests/camera_test.cpp)
TARGET_LINK_LIBRARIES(
    camera_test
    awesomo
    apriltags
    yaml-cpp
    flycapture
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
INSTALL(TARGETS camera_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# sim_test
# ADD_EXECUTABLE(sim_test tests/sim_test.cpp)
# TARGET_LINK_LIBRARIES(sim_test awesomo)
# INSTALL(TARGETS sim_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# pid_test
ADD_EXECUTABLE(pid_test tests/pid_test.cpp)
TARGET_LINK_LIBRARIES(pid_test awesomo)
INSTALL(TARGETS pid_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# util_test
ADD_EXECUTABLE(util_test tests/util_test.cpp)
TARGET_LINK_LIBRARIES(util_test awesomo)
INSTALL(TARGETS util_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# calibration images
ADD_EXECUTABLE(camera_calibration src/ros/camera_calibration_node.cpp)
TARGET_LINK_LIBRARIES( 
    camera_calibration
    awesomo
    apriltags
    yaml-cpp
    flycapture
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
