# Real-time HRSI Situation Classifier
A ROS kinetic package for realtime classification of situation in human-robot spatial interaction. To be used as part of the ILIAD warehouse robot software stack.

## Setup
Requires ROS kinetic, Ubuntu 16.04, and python3.5

Install the required apt packages:

`sudo apt install ros-kinetic-rosbridge-server python3-tk python3-venv`

Create a virtual environment to run the hmms library:
`pyenv venv-hmms`

`source venv-hmms/bin/activate`

`curl -fsSL -o- https://bootstrap.pypa.io/pip/3.5/get-pip.py | python3.5`

`python3.5 -m pip install roslibpy cython numpy`

`python3.5 -m pip install setuptools --upgrade`

`python3.5 -m pip install git+https://github.com/lopatovsky/HMMs`

Clone this repository in your ros workspace  directory of your catkin workspace and do `catkin_make`.

Finally, change the shebang of the `realtime_hrsi_situation_classifier/src/realttime_sit_classifier.py` script to use the python3.5 from the virtual environment created. E.g.: https://github.com/LCAS/iliad/blob/master/realtime_hrsi_situation_classifier/src/realtime_sit_classifier.py#L1

## Usage
Run the real-time classifier and its associated nodes using the provided launch file:
`roslaunch realtime_hrsi_situation_classifier realtime_sit_classifier.launch`

If necessary, set the appropriate robot ID with the robot_id argument ( default value is 5), e.g. `roslaunch realtime_hrsi_situation_classifier realtime_sit_classifier.launch robot_id:=4`
