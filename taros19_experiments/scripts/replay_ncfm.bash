#!/bin/bash

# Takes three parameters, experiment id, bag folder and ros workspace.
# Configures tmule and launches it

EXPERIMENT_ID=${1:-"S1-T1.1-A1"}
BAGS_FOLDER=${2:-"/home/tejas/BAGS-Tejas/"}
ROS_WORKSPACE=${3:-"/home/tejas/workspace/tj/"}

source $ROS_WORKSPACE"devel/setup.bash"

# you shouldn't need to change the following lines ...
cd `rospack find taros19_experiments`/tmule/
echo "----------------------------------------------------------------------"
echo "------------------- SYSTEM CONFIGURATION -----------------------------"
echo "Replaying scenario: ["$EXPERIMENT_ID"], at folder ["$BAGS_FOLDER"]"
echo "----------------------------------------------------------------------"
echo "-    *  SETTING ROS WORKSPACE ID IN yaml file                        -"
sed -i "s/ROS_WORKSPACE=.*/\ROS_WORKSPACE=\"${ROS_WORKSPACE//\//\\/}\"/g" ./replay_ncfm.yaml
echo "-    *  SETTING EXPERIMENT ID IN yaml file                           -"
sed -i "s/EXPERIMENT_ID=.*/\EEXPERIMENT_ID=\"$EXPERIMENT_ID\"/g" ./replay_ncfm.yaml
echo "-    *  SETTING BAG FOLDER IN yaml file                              -"
sed -i "s/BAGS_FOLDER=.*/\BAGS_FOLDER=\"${BAGS_FOLDER//\//\\/}\"/g" ./replay_ncfm.yaml
echo "----------------------------------------------------------------------"
echo "----------------    LAUNCHING TMULE SCRIPT    ------------------------"
tmule  --config replay_ncfm.yaml launch
echo "----------------------------------------------------------------------"
