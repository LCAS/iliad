cmake_minimum_required(VERSION 2.8.3)
project(iliad_smp_planner)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            nav_core
            base_local_planner
            pluginlib
            
            tf2_ros
            geometry_msgs
            tf2_geometry_msgs
        )

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES iliad_smp_planner
    CATKIN_DEPENDS
        roscpp
        pluginlib
        base_local_planner
        nav_core
)

add_library(iliad_smp_planner src/iliad_smp_planner.cpp)
add_dependencies(iliad_smp_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(iliad_smp_planner
    ${catkin_LIBRARIES}
    )

install(TARGETS iliad_smp_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


