cmake_minimum_required(VERSION 2.8.3)
project(moving_actor_gazebo)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})


catkin_package(
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY 
  launch
  config
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

