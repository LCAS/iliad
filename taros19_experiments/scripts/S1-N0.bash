#!/bin/bash
session_name="Human from right side. NO navigation "
TMULE_SCRIPT="S1-N0.yaml"

# you shouldn't need to change the following lines ...
cd `rospack find taros19_experiments`/tmule/
echo "----------------- SYSTEM CONFIGURATION ---------------------------"
echo "Scenario: ["$session_name"]"
echo "--------------    LAUNCHING TMULE SCRIPT    ----------------------"

tmule  --config $TMULE_SCRIPT launch
echo "------------------------------------------------------------------"
