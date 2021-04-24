cmake_minimum_required(VERSION 3.0.2)
project(bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)
find_package(OpenCV REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hw_bringup
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(pub_image src/pub_image.cpp)
target_link_libraries(pub_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(camera_sync src/camera_sync_test.cpp)
target_link_libraries(camera_sync ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})