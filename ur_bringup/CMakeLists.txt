
cmake_minimum_required(VERSION 3.0.2)
project(ur_bringup)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
  launch
  config
  ${catkin_INCLUDE_DIRS}
)