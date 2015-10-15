cmake_minimum_required(VERSION 2.8.3)
project(jsk_ur_startup)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)
