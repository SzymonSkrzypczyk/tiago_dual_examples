cmake_minimum_required(VERSION 3.10.2)
project(tiago_dual_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
  cv_bridge
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)