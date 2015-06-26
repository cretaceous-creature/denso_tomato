cmake_minimum_required(VERSION 2.8.3)
project(rgbtocloud)

set(CMAKE_BUILD_TYPE Release)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS tf pcl_ros pcl_conversions roscpp rospy std_msgs genmsg sensor_msgs)

find_package(PCL REQUIRED)
find_package(PCL 1.7 REQUIRED)
set(PCL_INCLUDE_DIRS "/usr/include/pcl-1.7/")
include_directories(${PCL_INCLUDE_DIRS})
include_directories("/usr/include/eigen3/")
include_directories("/usr/include/vtk-5.8/")
include_directories(include)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


## Generate messages in the 'msg' folder
# add_message_files(
#  DIRECTORY msg FILES Num.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   sensor_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also DIRECTORY msg FILES Num.msg
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rgbtocloud
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

#link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(PcProc src/pcproc.cpp)
#in this part the library file PcProc also needs to link to PCL_LIBRARY.
target_link_libraries(PcProc ${catkin_LIBRARIES}
				${PCL_LIBRARIES}
)
add_executable(listener src/listenopenni.cpp)
target_link_libraries(listener PcProc ${catkin_LIBRARIES}
					${PCL_LIBRARIES}
)
add_dependencies(listener $(catkin_EXPORTED_TARGETS))

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rgbtocloud.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
