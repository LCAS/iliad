cmake_minimum_required(VERSION 2.8.3)
project(gazebo_plugin_actor_collision)

find_package(catkin REQUIRED COMPONENTS
  #gazebo_plugins
  #gazebo_ros
  roscpp
)

find_package (gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${GAZEBO_INCLUDE_DIRS}
  LIBRARIES  ActorCollisionsPlugin
  CATKIN_DEPENDS roscpp #gazebo_plugins gazebo_ros
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS} 
                     ${Boost_INCLUDE_DIR} 
                     ${GAZEBO_INCLUDE_DIRS}
)

add_library(ActorCollisionsPlugin src/ActorCollisionsPlugin.cc)

