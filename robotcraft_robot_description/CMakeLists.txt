cmake_minimum_required(VERSION 2.8.3)
project(robotcraft_robot_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
  urdf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

