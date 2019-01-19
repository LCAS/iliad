#!/bin/bash
TMULE_SCRIPT="experiment.yaml"

# you shouldn't need to change the following lines ...
cd `rospack find taros19_experiments`/tmule/
echo "------------------------------------------------------------------"
echo "EVERYTHING IS BEING DESTROYED BY THE SHEER POWER OF MALEVOLENCE!  "
echo "------------------------------------------------------------------"
echo "-------------------- TERMINATING SYSTEM --------------------------"
echo "------------------------------------------------------------------"
tmule  --config $TMULE_SCRIPT terminate

echo "---------------- FORCING TMUX SESSION EXIT -----------------------"
tmux kill-session -t tmule

echo "------------------------------------------------------------------"
