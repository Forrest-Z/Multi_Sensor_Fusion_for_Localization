cmake_minimum_required(VERSION 3.0.2)
project(imu_trans)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  nodelet
  roscpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES imu_transformer_nodelet
  CATKIN_DEPENDS roscpp message_filters nodelet sensor_msgs tf2 tf2_ros geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_trans_node src/imu_transformer.cpp)
target_link_libraries(imu_trans_node ${catkin_LIBRARIES})