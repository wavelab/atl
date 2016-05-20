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
    libawesomo
    STATIC
    src/util.cpp
    src/controller.cpp
    src/ros/quadrotor.cpp
)

# EXECUTABLES
## ROS NODE EXECUTABLES
# quadrotor_node
ADD_EXECUTABLE(awesomo src/ros/awesomo.cpp)
TARGET_LINK_LIBRARIES(
    awesomo
    libawesomo
    apriltags
    yaml-cpp
    flycapture
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
INSTALL(
    TARGETS awesomo
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)





## UNIT TESTS
# tests/controller_test
ADD_EXECUTABLE(controller_test tests/controller_test.cpp)
TARGET_LINK_LIBRARIES(controller_test libawesomo yaml-cpp)
INSTALL(TARGETS controller_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# tests/util_test
ADD_EXECUTABLE(util_test tests/util_test.cpp)
TARGET_LINK_LIBRARIES(util_test libawesomo)
INSTALL(TARGETS util_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# tests/ros/quadrotor_node_test
ADD_EXECUTABLE(quadrotor_node_test tests/ros/quadrotor_node_test.cpp)
TARGET_LINK_LIBRARIES(
    quadrotor_node_test
    libawesomo
    apriltags
    yaml-cpp
    flycapture
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
INSTALL(
    TARGETS
    quadrotor_node_test
    RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# LAUNCH AND CONFIG FILES
INSTALL(
    DIRECTORY configs/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/configs
)

INSTALL(
    DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
