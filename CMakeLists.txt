cmake_minimum_required(VERSION 3.0.2)
project(tile_ground_segmentation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  pcl_ros
  pcl_conversions
  nav_msgs
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tile_ground_segmentation_node src/tile_ground_segmentation_node.cpp)
target_link_libraries(tile_ground_segmentation_node ${catkin_LIBRARIES})
