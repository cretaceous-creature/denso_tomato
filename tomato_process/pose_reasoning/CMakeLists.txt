cmake_minimum_required(VERSION 2.8.3)
project(pose_reasoning)

set(CMAKE_BUILD_TYPE Debug)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS pcl_ros pcl_conversions roscpp rospy std_msgs genmsg sensor_msgs)

find_package(PCL REQUIRED)
find_package(PCL 1.7 REQUIRED)
set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.7/")
include_directories(${PCL_INCLUDE_DIRS})
include_directories("/usr/include/eigen3/")
include_directories("/usr/include/vtk-5.8/")
include_directories(include)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   sensor_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# Display additional files in qtcreator
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
)


add_definitions(${PCL_DEFINITIONS})
add_library(MODELING src/modeling.cpp)

target_link_libraries(MODELING ${catkin_LIBRARIES}
			${PCL_LIBRARIES}
)
add_executable(pose_reasoning src/pose_reasoning.cpp)
target_link_libraries(pose_reasoning MODELING 
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
add_dependencies(pose_reasoning $(catkin_EXPORTED_TARGETS))

