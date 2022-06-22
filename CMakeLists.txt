cmake_minimum_required(VERSION 3.0.2)
project(pyhri)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  hri_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
  LIBRARIES pyhri
  CATKIN_DEPENDS cv_bridge geometry_msgs hri_msgs rospy sensor_msgs std_msgs tf2 tf2_ros
)

catkin_python_setup()

