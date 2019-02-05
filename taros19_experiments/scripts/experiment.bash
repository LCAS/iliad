#!/bin/bash

SCENARIO=${1:-1}
NAVIGATION=${2:-0}
endTime=${3:-16}

worldFileName="/home/manolofc/workspace/TAROS19/src/iliad/base_simulation/worlds/ncfm_model_1_actor_scenario-"$SCENARIO".world"

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
  Message="Robot drives into human."
  ;;
5)
    # special case: world naming not following pattern.
    Message="No human at all."
    worldFileName="/home/manolofc/workspace/TAROS19/src/iliad/base_simulation/worlds/ncfm_model_no_actors.world"
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

# Change human walking script duration or delay
#sed -i "s/.*sed marker 1.*/        \<delay_start\>"$endTime"\<\/delay_start\>  \<\!-- sed marker 1 --\>/" $worldFileName
sed -i "s/.*sed marker 2.*/            \<time\>"$endTime"\<\/time\>   \<\!-- sed marker 2 --\>/" $worldFileName


# you shouldn't need to change the following lines ...
cd `rospack find taros19_experiments`/tmule/
echo "------------------------------------------------------------------"
echo "----------------- SYSTEM CONFIGURATION ---------------------------"
echo "Scenario: ["$session_name"]"
echo "------------------------------------------------------------------"
echo "-  *  SETTING SCENARIO IN yaml file                              -"
sed -i "s/SCENARIO=.*/\SCENARIO=\"$SCENARIO\"/g" ./experiment.yaml
echo "-  *  SETTING NAVIGATION IN yaml file                            -"
sed -i "s/NAVIGATION=.*/\NAVIGATION=\"$NAVIGATION\"/g" ./experiment.yaml
echo "------------------------------------------------------------------"
echo "--------------    LAUNCHING TMULE SCRIPT    ----------------------"
tmule  --config experiment.yaml launch
echo "------------------------------------------------------------------"
