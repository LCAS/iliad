cmake_minimum_required(VERSION 2.8.3)
project(zombie)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  geometry_msgs
)


####################################
# Other parameters
####################################

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs 
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_library (SW_CANLIB canlib)


add_executable(zombie_node src/zombienode.cpp src/commandSender.cpp src/canlibWrapper.cpp)

target_link_libraries(zombie_node
   canlib
   ${catkin_LIBRARIES}
 )

#############
## Install ##
#############
