cmake_minimum_required(VERSION 2.8.3)
project(robotcraft_maze)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  stage_ros
  std_msgs
  tf
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pro_solver src/maze_PROSolver.cpp)

target_link_libraries(pro_solver ${catkin_LIBRARIES})
