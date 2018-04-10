# Human aware velocities
This package adjusts the velocity of the robot in the presence of humans.

## Human aware navigation
This node sets a navigation target and adjusts the velocity move_base/DWAPlannerROS uses to move the robot to a goal. This is achieved by using the 
dynamic reconfigure callback of the DWAPlannerROS to set the `max_vel_x`, `max_rot_vel`, and `max_trans_vel` according
to the distance of the robot to the closest human. As a consequence the robot will not be able to move linear or 
angular if a human is too close and will graduately slow down when approaching humans. The rotation speed is only 
adjusted to prevent the robot from spinning in place when not being able to move forward because this behaviour was 
observed even if the clearing rotation recovery behaviour is turnd off.

This is implemented as an action server.

### Parameters
* `pedestrian_locations` _Default: /pedestrian_localisation/localisations_: The topic on which the actual pedestrian 
locations are published by the pedestrian localisation package (bayes_people_tracker/PeopleTracker).
* `/human_aware_navigation/max_dist` _Default: 5.0_: maximum distance in metres to a human before altering the speed.
* `/human_aware_navigation/min_dist` _Default: 1.2_: minimum distance in metres to a human. Robot stops at that distance.
* `/human_aware_navigation/timeout` _Default: 5.0_: time in seconds after which speed is reseted if no human is detected any more.

### Running
```
rosrun strands_human_aware_velocity human_aware_planner_velocity.py
```
To start the actual functionality you have to use the actionlib client architecture, 
e.g. `rosrun actionlib axclient.py /human_aware_planner_velocities`
* goal type: move_base/MoveBaseGoal

## Human aware cmd_vel
A node to adjust the velocity of the robot by taking the /cmd_vel topic and republishing it. In order for this to work 
you have to remap the /cmd_vel output of your node or navigation stack to the input topic of this node. It relies on 
the output of the strands_pedestrian_localisation package to provide the actual position of humans in the vicinity of 
the robot.

### Parameters
* `pedestrian_location` _Default: /pedestrian_localisation/localisations_: The topic on which the actual pedestrian 
locations are published by the pedestrian localisation package (bayes_people_tracker/PeopleTracker).
* `cmd_vel_in` _Default: /human_aware_cmd_vel/input/cmd_vel_: The topic to which the original /cmd_vel should be 
published.
* `cmd_vel_out` _Default: /cmd_vel_: The modified /cmd_vel.
* `threshold` _Default: 3_: Threshold in seconds to determine which person detections from the cache are too old to use.
* `max_speed` _Default: 0.7_: The maximum speed the robot should achiev.
* `max_dist` _Default: 5.0_: The distance at which the node starts taking detections of humans into account.
* `min_dist` _Default: 1.5_: The cmd_vel will be 0.0 if there is a person closer to the robot than this.

### Running
```
rosrun strands_human_aware_velocity human_aware_cmd_vel [_parameter:=value]
```
