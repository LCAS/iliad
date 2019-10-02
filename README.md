THIS IS TIDY BRANCH:
This branch will eventually turn into master when all legacy stuff gets archived.

# Packages and description:
Note: Those packages are still in early development ("hic sunt dracones")

## gazebo_plugin_actor_collision
Actor collision plugin. Enables reporting collisions with actors, so that `gazebo-contactMonitor` can report those (see correspoding repo at https://github.com/LCAS/gazebo-contactMonitor)

## iliad_citi_controller
Extremely simple citi forklift controller. Contains a basic ROS/CAN bridge and a simple command converter. Command converter transforms from rot/trans vel commands (like a differential robot) to steering angle/speed and motor wheel speed.

## iliad_base_simulation
Gazebo world configurations and simulated citi forklifts used in other packages.

## iliad_oru_local_planer
Uses ROS local planners to perform local changes to orunav trajectories.

## iliad_dynamic_constraints
Contains several works regarding local alterations of speeds or constraints. 
`dynamic_constraints_node.py` captures trajectories from the execution node to the controller node and changes spatial and speed constraints.
`QSR2constraints_node.py` analyzes and plots in rviz current qtc state and publishes speed restrictions based on those.


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
