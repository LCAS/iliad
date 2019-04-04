#!/bin/bash

# Takes three parameters, experiment id, bag folder and ros workspace.
# Configures tmule and launches it

EXPERIMENT_ID=${1:-"S1-T1.1-A1"}
BAGS_FOLDER=${2:-"/home/tejas/BAGS-Tejas/"}
ROS_WORKSPACE=${3:-"/home/tejas/workspace/tj/"}
selected_uuid=${4:-""}
initial_px=${5:-"0.0"}
initial_py=${6:-"0.0"}
initial_oz=${7:-"0.0"}
initial_ow=${8:-"1"}

source $ROS_WORKSPACE"devel/setup.bash"

# you shouldn't need to change the following lines ...
cd `rospack find taros19_experiments`/tmule/
echo "----------------------------------------------------------------------"
echo "------------------- SYSTEM CONFIGURATION -----------------------------"
echo "Replaying scenario: ["$EXPERIMENT_ID"], at folder ["$BAGS_FOLDER"]"
echo "Using uuid: ["$selected_uuid"]"
echo "Start pose: ["$initial_px, $initial_py, $initial_oz, $initial_ow"]"
echo "----------------------------------------------------------------------"
echo "-    *  SETTING ROS WORKSPACE ID IN yaml file                        -"
sed -i "s/ROS_WORKSPACE=.*/\ROS_WORKSPACE=\"${ROS_WORKSPACE//\//\\/}\"/g" ./replay_ncfm.yaml
echo "-    *  SETTING UUID IN yaml file                                    -"
sed -i "s/selected_uuid=.*/\selected_uuid=\"${selected_uuid//\//\\/}\"/g" ./replay_ncfm.yaml
echo "-    *  SETTING EXPERIMENT ID IN yaml file                           -"
sed -i "s/EXPERIMENT_ID=.*/\EEXPERIMENT_ID=\"$EXPERIMENT_ID\"/g" ./replay_ncfm.yaml
echo "-    *  SETTING BAG FOLDER IN yaml file                              -"
sed -i "s/BAGS_FOLDER=.*/\BAGS_FOLDER=\"${BAGS_FOLDER//\//\\/}\"/g" ./replay_ncfm.yaml
echo "-    *  SETTING POSE IN yaml file                                    -"
sed -i "s/initial_px=.*/\initial_px=\"${initial_px//\//\\/}\"/g" ./replay_ncfm.yaml
sed -i "s/initial_py=.*/\initial_py=\"${initial_py//\//\\/}\"/g" ./replay_ncfm.yaml
sed -i "s/initial_oz=.*/\initial_oz=\"${initial_oz//\//\\/}\"/g" ./replay_ncfm.yaml
sed -i "s/initial_ow=.*/\initial_ow=\"${initial_ow//\//\\/}\"/g" ./replay_ncfm.yaml
echo "----------------------------------------------------------------------"
echo "----------------    LAUNCHING TMULE SCRIPT    ------------------------"
tmule  --config replay_ncfm.yaml launch
echo "----------------------------------------------------------------------"
