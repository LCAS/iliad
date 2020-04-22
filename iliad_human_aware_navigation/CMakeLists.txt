cmake_minimum_required(VERSION 2.8.3)
project(iliad_human_aware_navigation)

find_package(catkin REQUIRED COMPONENTS
   rospy
   tf
   tf2_ros
   message_filters
   bayes_people_tracker
   tf2_geometry_msgs
   orunav_msgs
   std_msgs
   visualization_msgs
   nav_msgs
   geometry_msgs
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
   CATKIN_DEPENDS 
   rospy
   tf
   tf2_ros
   message_filters
   bayes_people_tracker
   tf2_geometry_msgs
   orunav_msgs
   std_msgs
   visualization_msgs
   nav_msgs
   geometry_msgs
)

###########
## Build ##
###########


#############
## Install ##
#############

 install(PROGRAMS
   scripts/abort_and_replan.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(PROGRAMS
   scripts/iliad_constraints_costmap.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(PROGRAMS
   scripts/iliad_task_cost_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY rqtGui/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rqtGui
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)