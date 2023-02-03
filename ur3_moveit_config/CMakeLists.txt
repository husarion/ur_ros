cmake_minimum_required(VERSION 3.0.2)
project(ur3_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
  config
  launch
  urdf
  ${catkin_INCLUDE_DIRS}
)