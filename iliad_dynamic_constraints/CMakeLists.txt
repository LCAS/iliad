cmake_minimum_required(VERSION 2.8.3)
project(iliad_dynamic_constraints)

find_package(catkin REQUIRED COMPONENTS
#  dynamic_reconfigure
  roscpp
  rospy
  orunav_msgs
  driver_base
)

#generate_dynamic_reconfigure_options(
#   cfg/constraintSettings.cfg
#)

catkin_package(
#   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy orunav_msgs #dynamic_reconfigure 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(em_node src/em_node.cpp)

#add_dependencies(em_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(em_node
#   ${catkin_LIBRARIES}
# )

#install(TARGETS em_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)


## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/dynamic_constraints_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)