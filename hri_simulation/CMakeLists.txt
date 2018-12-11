cmake_minimum_required(VERSION 2.8.3)
project(hri_simulation)

find_package(catkin REQUIRED)

catkin_package(
)

#############
## Install ##
#############

install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
