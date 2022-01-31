#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray, Int64, Empty
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int64, Float64, Empty
from spencer_tracking_msgs.msg import DetectedPersons, DetectedPerson, TrackedPersons, TrackedPerson
import math
import tf.transformations
from orunav_msgs.msg import Task, ComputeTaskStatus,ReplanStatus, ControllerCommand
import json
import os

class control_experiments(object):
    def __init__(self):
        self.initial_experiment_number = 1
        self.last_experiment_numner = 20
        self.stop_experiment = False

        # Publishers
        rospy.Subscriber("stop_experiment",Empty,self.stop_experiment_callback,queue_size=1)
        print "starting experiments"

        rospy.sleep(10)

        for e in range(self.initial_experiment_number, self.last_experiment_numner+1):
            rospy.set_param('current_experiment', e)

            print "start experiment ",e
            os.system("tmule -c /home/sergi/catkin_ws/src/iliad_metapackage/tmule/iliad-coordinator.yaml launch -t all")
            os.system("tmule -c /home/sergi/catkin_ws/src/iliad_metapackage/tmule/iliad-robot.yaml launch")

            while self.stop_experiment ==False:
                print "waiting for experiment ",e," to finish"
                rospy.sleep(1)
            print "stop experiment ",e
            self.stop_experiment = False
            #wait for stop experiment to be published

            os.system("tmule -c /home/sergi/catkin_ws/src/iliad_metapackage/tmule/iliad-coordinator.yaml stop -t all")
            os.system("tmule -c /home/sergi/catkin_ws/src/iliad_metapackage/tmule/iliad-robot.yaml stop")

            rospy.sleep(10)

    def stop_experiment_callback(self,msg):
        self.stop_experiment = True


if __name__ == '__main__':
    rospy.init_node("control_experiments_node")#, log_level=rospy.DEBUG)
    try:
        ce = control_experiments()
    except rospy.ROSInterruptException:
        rospy.loginfo("Error -exception raised") 