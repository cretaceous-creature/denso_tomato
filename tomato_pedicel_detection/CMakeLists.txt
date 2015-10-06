cmake_minimum_required(VERSION 2.8.3)
project(tomato_pedicel_detection)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge jsk_perception)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS roscpp cv_bridge
   DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pedicel_detect src/pedicel_detect.cpp)
target_link_libraries(pedicel_detect ${catkin_LIBRARIES})

