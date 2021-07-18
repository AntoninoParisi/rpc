cmake_minimum_required(VERSION 3.0.2)
project(quality_inspection)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  urdf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gripper_mlp3240
#  CATKIN_DEPENDS roscpp rospy std_msgs urdf
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


