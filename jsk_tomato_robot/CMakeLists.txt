cmake_minimum_required(VERSION 2.8.3)
project(jsk_tomato_robot)

find_package(catkin REQUIRED COMPONENTS
  collada_urdf
  euscollada
)

catkin_package(
   INCLUDE_DIRS
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS collada_urdf euscollada
   DEPENDS
)

# set(_urdf_to_collada ${collada_urdf_PREFIX}/lib/collada_urdf/urdf_to_collada)
# set(_collada2eus ${euscollada_PREFIX}/lib/euscollada/collada2eus)

# message("urdf_to_collada: ${_urdf_to_collada}")
# message("collada2eus: ${_collada2eus}")

# if(NOT EXISTS ${PROJECT_SOURCE_DIR}/models/jsk_tomato_robot.l)
#   add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/jsk_tomato_robot.dae
#     COMMAND echo "${_urdf_to_collada} ${PROJECT_SOURCE_DIR}/models/jsk_tomato_robot.urdf jsk_tomato_robot.dae"
#     COMMAND ${_urdf_to_collada} ${PROJECT_SOURCE_DIR}/models/jsk_tomato_robot.urdf ${PROJECT_SOURCE_DIR}/models/jsk_tomato_robot.dae
#     DEPENDS ${PROJECT_SOURCE_DIR}/models/jsk_tomato_robot.urdf)
#   add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/models/jsk_tomato_robot.l
#     COMMAND echo "${_collada2eus} models/jsk_tomato_robot.dae models/jsk_tomato_robot.l"
#     COMMAND ${_collada2eus} ${PROJECT_SOURCE_DIR}/models/jsk_tomato_robot.dae ${PROJECT_SOURCE_DIR}/models/jsk_tomato_robot.yaml ${PROJECT_SOURCE_DIR}/models/jsk_tomato_robot.l
#     DEPENDS ${PROJECT_SOURCE_DIR}/models/jsk_tomato_robot.dae ${PROJECT_SOURCE_DIR}/models/jsk_tomato_robot.yaml ${_collada2eus})
#   add_custom_target(compile_jsk_tomato_robot ALL DEPENDS ${PROJECT_SOURCE_DIR}/models/jsk_tomato_robot.l)
# endif()

