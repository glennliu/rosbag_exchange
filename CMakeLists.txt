cmake_minimum_required(VERSION 3.0.2)
project(rosbag_exchange)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosbag
  geometry_msgs
  sensor_msgs
  nav_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosbag_reader_node src/rosbag_reader_node.cpp)
target_link_libraries(rosbag_reader_node
  ${catkin_LIBRARIES}
)
