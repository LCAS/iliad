cmake_minimum_required(VERSION 2.8.3)

## enable c++11 mode
set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

project(hanp_filters)

find_package(catkin REQUIRED COMPONENTS
  class_loader
#  hanp_msgs
  roscpp
  sensor_msgs
  tf
)

## install python modules and global scripts
# catkin_python_setup()

## add message, serviece and action files

# add_message_files( DIRECTORY msg
#   FILES
#     Message1.msg
# )

# add_service_files( DIRECTORY srv
#   FILES
#     Service1.srv
# )

# add_action_files( DIRECTORY action
#   FILES
#     Action1.action
# )

## generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   nav_msgs
#   sensor_msgs
#   std_msgs
# )

## add dynamic reconfigure configs
# generate_dynamic_reconfigure_options(
#   cfg/LaserFilter.cfg
# )

## declare catkin package
catkin_package(
  INCLUDE_DIRS
    include
  # LIBRARIES
  #   hanp_filters
  CATKIN_DEPENDS
#    hanp_msgs
    roscpp
    sensor_msgs
    tf
  # DEPENDS
  #   system_lib
)



## build ##

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## declare a c++ library
add_library(hanp_filters
  src/laser_filter.cpp
)

# cmake target dependencies of the c++ library
add_dependencies(hanp_filters ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# libraries to link the target c++ library against
target_link_libraries(hanp_filters
  ${catkin_LIBRARIES}
)

## declare a c++ executable
# add_executable(laser_filter
#   src/laser_filter.cpp
# )
#
# # cmake target dependencies of the c++ executable
# add_dependencies(laser_filter ${hanp_filters_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#
# # libraries to link the target c++ executable against
# target_link_libraries(laser_filter
#   ${catkin_LIBRARIES}
# )



## install ##

## executable scripts (python etc.) for installation
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## executables and/or libraries for installation
# install(TARGETS hanp_filters hanp_filters_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## cpp-header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## other files for installation (launch, data, bag files)
# foreach(dir launch data bags)
#    install(DIRECTORY ${dir}/
#       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

## other files for installation
# install(
#   FILES
#     myfile1
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )



## testing ##

## add catch based cpp test target
if(COMMAND catkin_add_catchtest)
  catkin_add_catchtest(${PROJECT_NAME}-test src/test/test_laser_filter.cpp)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

## add nosetest file folders
# catkin_add_nosetests(test)
