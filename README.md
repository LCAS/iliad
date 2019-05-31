# Packages and description:
Note: Those packages are still in early development ("hic sunt dracones")

## twist_controller (previously zombie)
Extremely simple forklift controller. Contains a basic ROS/CAN bridge and a simple command converter. Command converter transforms from rot/trans vel commands (like a differential robot) to steering angle/speed and motor wheel speed.

## base_simulation (previously in package named demo)
Gazebo world configurations and simulated forklifts used in other packages.

## nav_simulation (previously in package named demo)
Shows simulated forklift using ROS navigation stacks. Currently testing TEB planner and ROS DWA.

## hri_simulation
Depends on base and navigation simulation packages. It simulates human aware navigation.

## dynamic_constraints
Dynamic mpc constraints manager. Captures trajectories from the execution node to the controller node and modifies the spatial and speed constraints.

## gazebo_plugins
Actor collision plugin and binary contact sensor. The first is working nicely, the second was made redundant by contact_monitor (see correspoding repo at https://github.com/LCAS/gazebo-contactMonitor)

## iliad_human_local_navigation
Experiments for the human aware local navigation.

## iliad_launch
Deprecated. Global launchers for ILIAD project are now at gitlab repos.

## iliad_smp_planner
Luigi Palmieri already did this. This is a proof of concept of a ROS-compatible ILIAD planner.

## taros19_experiments
Launchers to create simulation experiments with same conditions but different navigation stacks. To be used in TAROS'19 paper.

## constraints maps
Provides a visualization of the mpc spatial constraints. 

## Source dependences:
Remember to source after building with ``catkin_make -DCMAKE_BUILD_TYPE=Release``

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
