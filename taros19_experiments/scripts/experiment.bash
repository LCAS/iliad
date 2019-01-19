#!/bin/bash

export SCENARIO=${1:-1}
export NAVIGATION=${2:-0}

case $SCENARIO in
1)
  Message="Robot crossess left of human."
  ;;
2)
  Message="Robot crossess right of human."
  ;;
3)
  Message="Robot crossess is overtaken by human."
  ;;
4)
  Message="Robot crossess crosses human"
  ;;
*)
  echo "Don't know Scenario "$SCENARIO"\n"
  exit
  ;;
esac

case $NAVIGATION in
1)
  Message=$Message" DWA Navigation."
  ;;
2)
  Message=$Message" TEB Navigation."
  ;;
3)
  Message=$Message" MPC Navigation."
  ;;
0)
  Message=$Message" NO Navigation - Possible collision."
  ;;
*)
  echo "Don't know Navigation "$NAVIGATION"\n"
  exit
  ;;
esac

session_name=$Message

# you shouldn't need to change the following lines ...
cd `rospack find taros19_experiments`/tmule/
echo "----------------- SYSTEM CONFIGURATION ---------------------------"
echo "Scenario: ["$session_name"]"
echo "--------------    LAUNCHING TMULE SCRIPT    ----------------------"

tmule  --config experiment.yaml launch
echo "------------------------------------------------------------------"
