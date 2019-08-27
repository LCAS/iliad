cmake_minimum_required(VERSION 2.8.3)
project(iliad_local_planner)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  tf2
  orunav_msgs
  pluginlib
  nav_core
  #geometry_msgs
  #sensor_msgs  
  costmap_2d
  base_local_planner
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES iliad_local_planner
  CATKIN_DEPENDS #roscpp  tf tf2 geometry_msgs sensor_msgs  nav_msgs
  #DEPENDS 
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  #${EIGEN3_INCLUDE_DIR}
)

add_executable(
   iliad_local_planner_node 
   src/iliad_local_planner_node.cpp
   src/iliad_local_planner.cpp
   )

## Specify libraries to link a library or executable target against
 target_link_libraries(
   iliad_local_planner_node
   ${Boost_LIBRARIES}
   ${catkin_LIBRARIES} 
 )

#############
## Install ##
#############

install(TARGETS iliad_local_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)
  
install(DIRECTORY images/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/images
  PATTERN ".svn" EXCLUDE)  

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  PATTERN ".svn" EXCLUDE)
  