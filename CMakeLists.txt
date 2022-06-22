cmake_minimum_required(VERSION 3.0.2)
project(viv_detection)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package( PCL REQUIRED )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES viv_detection
#  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)



link_directories(${PCL_LIBRARY_DIRS} ${message_filters_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(detection_node src/detection_node.cpp)
target_link_libraries(detection_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})