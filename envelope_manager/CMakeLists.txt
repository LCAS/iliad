cmake_minimum_required(VERSION 2.8.3)
project(envelope_manager)

find_package(catkin REQUIRED
  cmake_modules
  roscpp
  orunav_msgs
)


catkin_package(CATKIN_DEPENDS 
                    roscpp 
                    orunav_msgs 
               DEPENDS 
                    Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(envelope_manager_node src/envelope_manager_node.cpp)
add_dependencies(envelope_manager_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(envelope_manager_node
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS envelope_manager_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

