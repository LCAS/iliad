# Packages and description:
Note: Those packages are still in early development ("hic sunt dracones")


## twist_controller (previously zombie)
Extremely simple forklift controller. Contains a basic ROS/CAN bridge and a simple command converter. Command converter transforms from rot/trans vel commands (like a differential robot) to steering angle/speed and motor wheel speed. 

## base_simulation (previously in package named demo)
Gazebo world configurations and simulated forklifts used in other packages.

## nav_simulation (previously in package named demo)
Shows simulated forklift using ROS navigation stacks. Currently testing TEB planner and ROS DWA.

## hri_simulation
Depends on base and navigatio simulation packages. It simulates human aware navigation. 

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

