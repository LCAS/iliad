#!/bin/bash

# Batch launcher for simulation experiment tmule script.
# Creates the bags for the specified bag filenames


for ((SCENARIO = 1; SCENARIO < 6; SCENARIO++)); do
  for ((NAVIGATION = 0; NAVIGATION < 4; NAVIGATION++)); do
    for ((ATTEMPT = 1; ATTEMPT < 4; ATTEMPT++)); do
      echo -e "Executing S$SCENARIO-N$NAVIGATION attempt number $ATTEMPT"
      rosrun taros19_experiments experiment.bash $SCENARIO $NAVIGATION
      # max time x 2
      sleep 120
      echo -e "done."
      sleep 3
      echo -e "Killing and cleaning."
      rosrun taros19_experiments kill-tmule.bash
      sleep 5
      echo -e "done."
      echo -e "Wait 5 secs so you can try read this."
      sleep 5
      echo -e "done."
    done
      echo -e ""
  done
done
