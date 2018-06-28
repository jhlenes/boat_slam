cmake_minimum_required(VERSION 2.8.3)
project(boat_slam)

find_package(catkin REQUIRED COMPONENTS
  cartographer_ros
  rplidar_ros
  roscpp
  rospy
  tf2
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES boat_slam
#  CATKIN_DEPENDS cartographer_ros rplidar_ros roscpp rospy tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(drive_controller_node src/drive_controller.cpp)

target_link_libraries(drive_controller_node
  ${catkin_LIBRARIES}
)

#Add custom (non compiling) targets so launch scripts and python files show up in QT Creator's project view.
file(GLOB_RECURSE EXTRA_FILES */*)
add_custom_target(${PROJECT_NAME}_OTHER_FILES ALL WORKING_DIRECTORY ${PROJECT_SOURCE_DIR} SOURCES ${EXTRA_FILES})
