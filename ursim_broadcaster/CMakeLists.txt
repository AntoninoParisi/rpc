cmake_minimum_required(VERSION 3.0.2)
project(ursim_broadcaster)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_core
  moveit_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_runtime
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  trajectory_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ursim_broadcaster
#  CATKIN_DEPENDS geometry_msgs moveit_core moveit_msgs roscpp rospy std_msgs
CATKIN_DEPENDS message_runtime std_msgs sensor_msgs trajectory_msgs
)



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ursim_broadcaster src/broadcaster.cpp)
target_link_libraries(ursim_broadcaster ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
