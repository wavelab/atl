cmake_minimum_required(VERSION 2.8.3)
project(awesomo)

# INCLUDES
include_directories(
    ${catkin_INCLUDE_DIRS}
    /usr/local/include
    /usr/include/eigen3
    ./include
)

# LIBS
link_directories(
    /usr/local/lib
    /usr/lib
)


# DEPENDENCIES
find_package(
    catkin
    REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
)
find_package(OpenCV)

# BUILD AS CATKIN PACKAGE
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES awesomo
    CATKIN_DEPENDS roscpp rospy std_msgs
    DEPENDS system_lib
)

# EXECUTABLES
add_executable(core src/core.cpp)
target_link_libraries(core ${catkin_LIBRARIES})
install(
    TARGETS core
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(camera src/camera.cpp)
target_link_libraries(
    camera
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    apriltags
)
install(
    TARGETS camera
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
