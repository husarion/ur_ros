
cmake_minimum_required(VERSION 3.0.2)
project(ur_components_description)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
  meshes
  urdf
  ${catkin_INCLUDE_DIRS}
)