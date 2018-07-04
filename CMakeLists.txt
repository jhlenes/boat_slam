cmake_minimum_required(VERSION 2.8.3)
project(boat_slam)

find_package(catkin REQUIRED COMPONENTS
  cartographer_ros
  rplidar_ros
  roscpp
  rospy
  tf2
  tf2_ros
  nav_core
  costmap_2d
  base_local_planner
  pluginlib
)

include_directories(${catkin_INCLUDE_DIRS})

## Add custom (non compiling) targets so other files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})

## Declare a catkin package
catkin_package()

## Add nodes
add_executable(map_receiver_node src/map_receiver.cpp)
target_link_libraries(map_receiver_node ${catkin_LIBRARIES})

## Add global path planner plugin
add_library(global_planner src/global_planner.cpp)

