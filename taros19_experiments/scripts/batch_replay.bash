#!/bin/bash

BAGS_FOLDER="/home/manolofc/iliad/tj/"
ROS_WORKSPACE="/home/manolofc/workspace/taros/"
SAVE_FOLDER="/home/manolofc/iliad/NCFM_HUMAN_BAGS_processed/"

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

## now loop through the above array
for i in "${experiment_ids[@]}"
do
   echo -e "......................"
   echo "Replaying file: "$i
   rosrun taros19_experiments replay_ncfm.bash $i $BAGS_FOLDER $ROS_WORKSPACE
   # max time x 2
   sleep 120
   echo -e "done."
   sleep 3
   echo -e "Killing and cleaning."
   rosrun taros19_experiments kill-tmule.bash replay_ncfm.yaml
   sleep 5
   echo -e "done."
   echo -e "Moving bag file to ["$SAVE_FOLDER"]"
   mv *.bag $SAVE_FOLDER
   echo -e "done."
   echo -e "......................"
   echo -e "Wait 5 secs so you can try read this."
   sleep 5
   echo -e "done."
   echo -e "......................\n\n\n"
done
