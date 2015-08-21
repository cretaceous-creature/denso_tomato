cmake_minimum_required(VERSION 2.8.3)
project(ureus)

find_package(catkin REQUIRED COMPONENTS
  collada_urdf
  euscollada
  ur_description
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS collada_urdf euscollada
  DEPENDS
)

if(EXISTS ${ur_description_SOURCE_PREFIX}/urdf)
  set(_ur_urdf ${ur_description_SOURCE_PREFIX}/urdf)
else()
  set(_ur_urdf ${ur_description_PREFIX}/share/ur_description/urdf)
endif()
set(_urdf_to_collada ${collada_urdf_PREFIX}/lib/collada_urdf/urdf_to_collada)
set(_collada2eus ${euscollada_PREFIX}/lib/euscollada/collada2eus)

message("ur_urdf: ${_ur_urdf}")
message("urdf_to_collada: ${_urdf_to_collada}")
message("collada2eus: ${_collada2eus}")


if(NOT EXISTS ${PROJECT_SOURCE_DIR}/ur5_robot.l)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/ur5_robot.urdf
    COMMAND cp ${_ur_urdf}/ur5_robot.urdf ${PROJECT_SOURCE_DIR}/models/ur5_robot.urdf
    DEPENDS ${_ur_urdf}/ur5_robot.urdf)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/ur5_robot.dae
    COMMAND echo "${_urdf_to_collada} ${PROJECT_SOURCE_DIR}/ur5_robot.urdf ur5_robot.dae"
    COMMAND ${_urdf_to_collada} ${PROJECT_SOURCE_DIR}/models/ur5_robot.urdf ${PROJECT_SOURCE_DIR}/models/ur5_robot.dae
    DEPENDS ${PROJECT_SOURCE_DIR}/models/ur5_robot.urdf)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/ur5_robot.l
    COMMAND echo "${_collada2eus} models/ur5_robot.dae models/ur5_robot.l"
    COMMAND ${_collada2eus} ${PROJECT_SOURCE_DIR}/models/ur5_robot.dae ${PROJECT_SOURCE_DIR}/models/ur5_robot.yaml ${PROJECT_SOURCE_DIR}/models/ur5_robot.l
    DEPENDS ${PROJECT_SOURCE_DIR}/models/ur5_robot.dae ${PROJECT_SOURCE_DIR}/models/ur5_robot.yaml ${_collada2eus})
  add_custom_target(compile_ur ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/ur5_robot.l)
endif()

