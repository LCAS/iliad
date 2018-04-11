cmake_minimum_required(VERSION 2.8.3)
project(hrsi_representation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  bayes_people_tracker
  dynamic_reconfigure
  geometry_msgs
  message_generation
  nav_msgs
  qsr_lib
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  QTC.msg
  QTCArray.msg
)

generate_dynamic_reconfigure_options(
  cfg/OnlineQTCCreator.cfg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/offline_qtc_creator.py
  scripts/online_qtc_creator.py
  scripts/goal_pose_republisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# if (CATKIN_ENABLE_TESTING)
#   find_package(catkin REQUIRED COMPONENTS rostest)
#   add_rostest(tests/online_creator_tester.test)
#   add_rostest(tests/offline_creator_tester.test)
# endif()
