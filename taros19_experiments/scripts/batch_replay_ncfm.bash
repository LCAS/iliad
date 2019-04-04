#!/bin/bash

# Batch launcher for replay_ncfm tmule script.
# Replays the given bags, gets the necessary topics and
# creates smaller bags with the results


BAGS_FOLDER="/home/manolofc/iliad/tj/"
ROS_WORKSPACE="/home/manolofc/workspace/taros/"
SAVE_FOLDER="/home/manolofc/iliad/NCFM_HUMAN_BAGS_processed3/"

declare -a experiment_ids=("S1-T1.1-A1"   # 0
                           "S1-T1.1-A2"   # 1
                           "S1-T1.2-A1"   # 2
                           "S1-T1.2-A2"   # 3
                           "S1-T2.1-A1"   # 4
                           "S1-T2.1-A2"   # 5
                           "S1-T2.2-A1"   # 6
                           "S1-T2.2-A2"   # 7
                           "S1-T3.1-A1"   # 8
                           "S1-T3.1-A2"   # 9
                           "S1-T3.2-A1"   # 10
                           "S1-T3.2-A2"   # 11
                           "S1-T4.1-A1"   # 12
                           "S1-T4.1-A2"   # 13
                           "S1-T4.2-A1"   # 14
                           "S1-T4.2-A2"   # 15
                           )
declare -a uuids=("0"  # No detections!
                  "0"
                  "2"
                  "2"
                  "5"
                  "42" # No detections ....
                  "0"
                  "1"
                  "1"
                  "1"
                  "0"
                  "0"
                  "2"
                  "0"
                  "0"
                  "0"
                  )
 declare -a initial_poses=("1.05 -0.305 0.9828 0.184"
                           "1.05 -0.305 0.9828 0.184"
                           "1.05 -0.305 0.9828 0.184"
                           "1.05 -0.305 0.9828 0.184"
                           "1.05 -0.305 0.9828 0.184"
                           "1.05 -0.305 0.9828 0.184"
                           "-9.06 -3.45 -0.81 0.58"   # 6
                           "-9.06 -3.45 -0.81 0.58"   # 7 short
                           "1.05 -0.305 0.9828 0.184" # 8
                           "1.05 -0.305 0.9828 0.184" # 9
                           "-9.06 -3.45 -0.81 0.58"   # 10
                           "-9.06 -3.45 -0.81 0.58"   # 11
                           "1.05 -0.305 0.9828 0.184" # 12
                           "1.05 -0.305 0.9828 0.184" # 13
                           "-9.06 -3.45 -0.81 0.58"   # 14 move back!
                           "-9.06 -3.45 -0.81 0.58"   # 15 move back!
                           )

source $ROS_WORKSPACE"devel/setup.bash"





# you shouldn't need to change the following lines ...
cd `rospack find taros19_experiments`/tmule/

## now loop through the above array
for i in {12..15} #{0..15}
do
   file=${experiment_ids[i]}
   initial_pose=${initial_poses[i]}
   uuid=${uuids[i]}
   initial_px=${initial_pose[0]}
   initial_py=${initial_pose[1]}
   initial_oz=${initial_pose[2]}
   initial_ow=${initial_pose[3]}
   echo -e "......................"
   echo "Replaying file ("$i"): "$file
   rosrun taros19_experiments replay_ncfm.bash $file $BAGS_FOLDER $ROS_WORKSPACE $uuid $initial_px $initial_py $initial_oz $initial_ow
   # max time x 2
   sleep 25
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
