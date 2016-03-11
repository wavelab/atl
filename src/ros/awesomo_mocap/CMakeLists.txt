cmake_minimum_required(VERSION 2.8.3)
project(awesomo_mocab)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtle_parrot
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
 add_executable(awesomo_mocap src/awesomo_mocap.cpp)
target_link_libraries(awesomo_mocap
   ${catkin_LIBRARIES}
 )

