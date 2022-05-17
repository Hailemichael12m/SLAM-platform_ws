cmake_minimum_required(VERSION 3.0.2)
project(frontier_description)

find_package(catkin REQUIRED COMPONENTS
  realsense2_description  # To throw a failure if the realsense2_description is not installed
  xacro
)

catkin_package(CATKIN_DEPENDS xacro)


install(DIRECTORY config launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
