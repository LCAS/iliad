cmake_minimum_required(VERSION 2.8.3)
project(envelope_manager)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  orunav_msgs
)

generate_dynamic_reconfigure_options(
   cfg/envelopeManager.cfg
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS dynamic_reconfigure roscpp rospy orunav_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(em_node src/em_node.cpp)

add_dependencies(em_node ${PROJECT_NAME}_gencfg ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(em_node
   ${catkin_LIBRARIES}
 )

install(TARGETS em_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
