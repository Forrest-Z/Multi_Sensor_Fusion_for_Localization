cmake_minimum_required(VERSION 3.0.2)
project(ros_detection_tracking)

set(OpenCV_DIR "/home/base/shared_dir/opencv-3.4.12/build")

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
  dynamic_reconfigure
)
find_package(OpenCV REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_detection_tracking
#  CATKIN_DEPENDS cv_bridge pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
 #include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


# add_executable(detectandtrack src/ros_ttc/detectandtrackNode.cpp src/ros_ttc/camFusion_Student.cpp src/ros_ttc/lidarData.cpp
# src/ros_ttc/matching2D_Student.cpp src/ros_ttc/objectDetection2D.cpp)
# target_link_libraries(detectandtrack ${catkin_LIBRARIES}  ${OpenCV_LIBS})
# add_dependencies(detectandtrack ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(projector src/project/project.cpp)
target_link_libraries(projector ${catkin_LIBRARIES}
                               ${OpenCV_LIBS})
add_dependencies(projector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
