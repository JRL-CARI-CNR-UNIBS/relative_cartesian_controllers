cmake_minimum_required(VERSION 2.8.3)
project(relative_cartesian_controller_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  geometry_msgs
  actionlib_msgs
  std_msgs
)


add_action_files( DIRECTORY 	action
  FILES
  RelativeMove.action
)

generate_messages( DEPENDENCIES actionlib_msgs geometry_msgs std_msgs)

catkin_package(
   CATKIN_DEPENDS actionlib_msgs geometry_msgs message_generation std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
