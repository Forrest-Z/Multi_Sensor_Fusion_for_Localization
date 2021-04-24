cmake_minimum_required(VERSION 3.0.2)
project(cam_lidar_seg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  image_transport
  message_generation
  cmake_modules
)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_detection_tracking
#  CATKIN_DEPENDS cv_bridge pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(projector src/project.cpp)
target_link_libraries(projector ${catkin_LIBRARIES}
                                ${OpenCV_LIBS})
add_dependencies(projector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(fusion src/fusion.cpp)
target_link_libraries(fusion ${catkin_LIBRARIES}
                                ${OpenCV_LIBS})
add_dependencies(fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})