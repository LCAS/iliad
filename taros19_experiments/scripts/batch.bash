#!/bin/bash


# source here


for ((SCENARIO = 1; SCENARIO < 5; SCENARIO++)); do
  for ((NAVIGATION = 1; NAVIGATION < 4; NAVIGATION++)); do
    for ((ATTEMPT = 1; ATTEMPT < 4; ATTEMPT++)); do
      echo -e "Executing S$SCENARIO-N$NAVIGATION attempt number $ATTEMPT"
      rosrun taros19_experiments experiment.bash $SCENARIO $NAVIGATION
      echo -e "done."
      sleep 3
      echo -e "Killing and cleaning."
      rosrun taros19_experiments kill-tmule.bash
      echo -e "done."
      echo -e "Wait 5 secs so you can try read this."
      sleep 5
      echo -e "done."
    done
      echo -e ""
  done
done
