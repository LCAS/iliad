# Packages and description:
Note: Those packages are still under heavy development ("hic sunt dracones")

## gazebo_plugin_actor_collision
Actor collision plugin. Enables reporting collisions with actors, so that `gazebo-contactMonitor` can report those (see correspoding repo at https://github.com/LCAS/gazebo-contactMonitor)

## iliad_base_simulation
Gazebo world configurations and simulated citi forklifts used in other packages.

## iliad_citi_controller
Extremely simple citi forklift controller. Contains a basic ROS/CAN bridge and a simple command converter. Command converter transforms from rot/trans vel commands (like a differential robot) to steering angle/speed and motor wheel speed.

## iliad_hrsi
Contains several works regarding human robot spatial interaction in iliad. 

## iliad_human_aware_navigation
Nodes that take hrsi outputs and (potentially) change the way ILIAD robot navigates. 

## iliad_oru_local_planer
Uses ROS local planners to perform local changes to orunav trajectories.

## Other relevant packages:

* strands_hri
https://github.com/strands-project/strands_hri

* strands_perception_people
https://github.com/strands-project/strands_perception_people

* strands_navigation
https://github.com/strands-project/navigation/tree/han_dwa_only_kinetic

* strands_qsr_lib
https://github.com/strands-project/strands_qsr_lib

* ghmm (required by qsr_lib)
http://ghmm.sourceforge.net/installation.html

* velodyne_simulator
https://github.com/LCAS/velodyne_simulator

* kinect2_simulation
https://gitsvn-nt.oru.se/iliad/software/kinect2_simulation

* iai_kinect2
https://github.com/code-iai/iai_kinect2

* navigation_oru
https://github.com/OrebroUniversity/navigation_oru-release

* Spencer people tracking (iliad branch)
https://github.com/LCAS/spencer_people_tracking/tree/ilidad-dev

* TF utilities for BAG files
https://github.com/LCAS/tf_bag
