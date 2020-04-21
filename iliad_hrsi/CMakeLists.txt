cmake_minimum_required(VERSION 2.8.3)
project(iliad_hrsi)

find_package(catkin REQUIRED COMPONENTS
  rospy
  bayes_people_tracker  
  tf
  tf2_ros
  orunav_msgs
  geometry_msgs
  std_msgs
  spencer_tracking_msgs
  visualization_msgs
  tf2_geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS 
  rospy
  bayes_people_tracker  
  tf
  tf2_ros
  orunav_msgs
  geometry_msgs
  std_msgs
  spencer_tracking_msgs
  visualization_msgs
  tf2_geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/QTC_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/QTCplot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/spencer_to_bayes.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)