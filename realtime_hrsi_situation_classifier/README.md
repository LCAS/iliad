# Real-time HRSI Situation Classifier
A ROS kinetic package for realtime classification of situation in human-robot spatial interaction. To be used as part of the ILIAD warehouse robot software stack.

## Setup
Requires ROS kinetic, Ubuntu 16.04, and python3.5

Install the required apt packages:

`sudo apt install cython3 ros-kinetic-strands-qsr-lib python3-numpy python3-pandas python3-scipy`

Install the required pip packages:
`python3.5 -m pip install roslibpy hmms`

If `hmms` from pypi fails to install, install from the GitHub repo:
```
git clone https://github.com/lopatovsky/HMMs.git
cd HMMs
pip install -e .
```

Finally, clone this repository into the `src` directory of your catkin workspace and call `catkin build realtime_hrsi_situation_classifier`.

## Usage
Run the real-time classifier and its associated nodes using the provided launch file:
`roslaunch realtime_hrsi_situation_classifier realtime_sit_classifier.launch`

If necessary, set the appropriate robot ID with the robot_id argument ( default value is 5), e.g. `roslaunch realtime_hrsi_situation_classifier realtime_sit_classifier.launch robot_id:=4`