cmake_minimum_required(VERSION 3.0.2)
project(micasense_camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()