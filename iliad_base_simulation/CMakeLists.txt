cmake_minimum_required(VERSION 2.8.3)
project(iliad_base_simulation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

catkin_package(
  CATKIN_DEPENDS
  gazebo_ros
)

#############
## Install ##
#############

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


set(MODELS_ZIPFILE orkla_warehouse.zip)
set(MODELS_DEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/models)
set(MODELS_DOWNLOAD_URL https://lcas.lincoln.ac.uk/nextcloud/index.php/s/9nJLNryeyLsF4Fg/download)

add_custom_command(OUTPUT ${MODELS_ZIPFILE}
  COMMAND curl --progress-bar -o "${MODELS_ZIPFILE}" "${MODELS_DOWNLOAD_URL}"
  COMMENT "downloading additional models"
  VERBATIM
)

add_custom_command(
  OUTPUT .models-extracted
  DEPENDS ${MODELS_ZIPFILE}
  COMMAND unzip -q -o -d "${MODELS_DEST_DIR}" "${MODELS_ZIPFILE}"
  COMMAND touch .models-extracted
  COMMENT "extracting additional models"
  VERBATIM
)

add_custom_target(get_models ALL
  DEPENDS 
  .models-extracted
)

add_dependencies(get_models preinstall)