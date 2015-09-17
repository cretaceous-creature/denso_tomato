cmake_minimum_required(VERSION 2.8.3)
project(tomato_fruits_ripeness)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES
   CATKIN_DEPENDS roscpp cv_bridge
   DEPENDS
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tomato_extractor src/tomato_extractor.cpp)
target_link_libraries(tomato_extractor ${catkin_LIBRARIES})

