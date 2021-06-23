# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.0.2)
project(imu_trans)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED roscpp sensor_msgs geometry_msgs nav_msgs tf)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs tf
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

add_executable(imu_trans_node src/imu_trans_node.cpp)
target_link_libraries(imu_trans_node ${catkin_LIBRARIES})

