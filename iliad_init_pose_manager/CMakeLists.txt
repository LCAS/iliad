cmake_minimum_required(VERSION 2.8.3)
project(iliad_init_pose_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
  geometry_msgs
  rospy_message_converter
)


catkin_package(
  CATKIN_DEPENDS 
  rospy
  nav_msgs
  geometry_msgs
  rospy_message_converter
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/init_pose_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)