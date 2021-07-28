cmake_minimum_required(VERSION 3.0.2)
project(quality_inspection)


find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface 
  moveit_ros_move_group
  moveit_visual_tools
  std_msgs
  geometry_msgs
  message_generation
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
    # INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(quality_inspection_node src/quality_inspection.cpp)
target_link_libraries(quality_inspection_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

