#!/bin/bash

# creates NDT maps from bags


BAGS_FOLDER="/home/manolofc/iliad/tj/"
ROS_WORKSPACE="/home/manolofc/workspace/taros/"
SAVE_FOLDER="/home/manolofc/iliad/tj/"

declare -a experiment_ids=("S1-T1.1-A1"
                              "S1-T1.1-A2"
                              "S1-T1.2-A1"
                              "S1-T1.2-A2"
                              "S1-T2.1-A1"
                              "S1-T2.1-A2"
                              "S1-T2.2-A1"
                              "S1-T2.2-A2"
                              "S1-T3.1-A1"
                              "S1-T3.1-A2"
                              "S1-T3.2-A1"
                              "S1-T3.2-A2"
                              "S1-T4.1-A1"
                              "S1-T4.1-A2"
                              "S1-T4.2-A1"
                              "S1-T4.2-A2"
                              )

source $ROS_WORKSPACE"devel/setup.bash"

# you shouldn't need to change the following lines ...
cd `rospack find taros19_experiments`/tmule/

# now loop through the above array
for i in {0..15}
do
  file=${experiment_ids[i]}
  echo -e "......................"
  echo "Replaying file: "$file
  roslaunch iliad_launch_system fuser_iliad_velodyne16.launch robot_id:=5 &
  sleep 5
  rosbag play --clock  $BAGS_FOLDER$file".bag"
  echo -e "done."
  sleep 3

  echo -e "......................"
  echo "saving ndt map: "
  rosservice call /fuser_3d/save_map "{}"
  sleep 3
  echo -e "done."
  sleep 3

  echo -e "......................"
  echo -e "Moving map file to ["$SAVE_FOLDER"]"
  mv $ROS_WORKSPACE"src/iliad_metapackage/maps/iliad_map.jff"  $SAVE_FOLDER$file".jff"
  echo -e "done."
  sleep 3

  echo -e "......................"
  echo -e "Killing and cleaning."
  pkill roslaunch
  pkill roscore
  pkill rosmaster
  sleep 5
  echo -e "done."
  echo -e "......................\n\n\n"
done
