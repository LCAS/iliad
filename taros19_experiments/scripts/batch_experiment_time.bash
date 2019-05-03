#!/bin/bash

# Like batch.sh but now we specify a third param with the human target end time.


declare -a ScenarioNavigationTime=(
                      # Scenario 1 Robot crosses human left to right
                      #no collisions
                      "1 0 13"
                      "1 1 16"
                      "1 2 16"
                      "1 3 16"
                      #FORCING collisions
                      "1 0 14"
                      "1 1 29"
                      "1 2 28"
                      "1 3 28"
                      # Scenario 2 Robot crosses human right to left
                      #no collisions
                      "2 0 16"
                      "2 1 16"
                      "2 2 16"
                      "2 3 16"
                      #FORCING collisions
                      "2 0 17"
                      "2 1 37"
                      "2 2 37"
                      "2 3 37"
                      # Scenario 3 Robot is overtaken by human
                      #no collisions
                      "3 0 12"
                      "3 1 12"
                      "3 2 12"
                      "3 3 12"
                      # Scenario 4 Robot is headed to human
                      #no collisions
                      "4 0 35" # will collide anyways
                      "4 1 35"
                      "4 2 35"
                      "4 3 35" # Human will collide always
                      #FORCING collisions
                      "4 0 12"
                      "4 1 12"
                      "4 2 12"
                      "4 3 12"
                      # Scenario 5 Robot is alone
                      #no collisions
                      "5 0 16"
                      "5 1 16"
                      "5 2 16"
                      "5 3 16"
          )


ROS_WORKSPACE="/home/manolofc/workspace/taros/"
SAVE_FOLDER="/home/manolofc/iliad/TAROS19_sims/new_bags/"

source $ROS_WORKSPACE"devel/setup.bash"
cd `rospack find taros19_experiments`/tmule/

for ((ATTEMPT = 1; ATTEMPT < 4; ATTEMPT++)); do
      ## now loop through the above array
      for i in "${ScenarioNavigationTime[@]}"
      do
         conf=( $i )
         s=${conf[0]}
         n=${conf[1]}
         t=${conf[2]}
         a=$ATTEMPT
         echo -e "......................"
         echo "Running simulation: Scenario: "$s", Navigation: "$n", Human endTime: "$t", Attempt: "$a
         rosrun taros19_experiments experiment.bash $i $ROS_WORKSPACE
         sleep 150
         echo -e "done."
         echo -e "......................"
         echo -e "Killing and cleaning."
         rosrun taros19_experiments kill-tmule.bash
         sleep 1
         echo -e "done."
         echo -e "......................"
         echo -e "Moving bag file to ["$SAVE_FOLDER"]"
         mv *.bag $SAVE_FOLDER
         sleep 1
         echo -e "done."
         echo -e "......................"
         echo -e "Wait 5 secs so you can try read this."
         sleep 5
         echo -e "done."
         echo -e "......................\n\n\n"
      done
done
