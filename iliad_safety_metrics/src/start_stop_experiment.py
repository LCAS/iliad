#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int64, Float64, Empty
from spencer_tracking_msgs.msg import DetectedPersons, DetectedPerson, TrackedPersons, TrackedPerson
import math
import tf.transformations
from orunav_msgs.msg import Task, ComputeTaskStatus,ReplanStatus, ControllerCommand
import json


class start_stop_experiments(object):
    def __init__(self):

        # Publishers
        self.start_experiment_pub = rospy.Publisher("start_experiment", Empty, queue_size=1)
        self.stop_experiment_pub = rospy.Publisher("stop_experiment", Empty, queue_size=1)

        rospy.Subscriber("recording_finished",Empty,self.recording_finished_callback,queue_size=1)
        rospy.Subscriber('/robot1/compute_task/status', ComputeTaskStatus, self.compute_task_callback, queue_size=1)

        rospy.sleep(25)
        self.start_experiment_pub.publish(Empty())
        print "experiment started"

        rospy.sleep(1000)
        self.stop_experiment_pub.publish(Empty())
        print "experiment finished"

        rospy.spin()

    def recording_finished_callback(self,msg):
        self.stop_experiment_pub.publish(Empty())
        print "experiment finished"

    def compute_task_callback(self,msg): #this is just for debugging purposes
        if msg.status == 0:
            message = "COMPUTE_TASK_START"
        if msg.status == 1:
            message = "COMPUTE_TASK_SUCCESS"
        if msg.status == 2: 
            message = "INVALID_TARGET"
        if msg.status == 3:
            message = "INVALID_MAP"
        if msg.status == 4:
            message = "INVALID_START"
        if msg.status == 5: 
            message = "VECTOR_MAP_SERVICE_SUCCESS"
        if msg.status == 6:
            message = "VECTOR_MAP_SERVICE_FAILURE"
            self.stop_experiment_pub.publish(Empty())
        if msg.status == 7:
            message = "GEOFENCE_CALL_SUCCESS"
        if msg.status == 8: 
            message = "GEOFENCE_CALL_FAILURE"
            self.stop_experiment_pub.publish(Empty())
        if msg.status == 9:
            message = "PATH_PLANNER_SERVICE_SUCCESS"
        if msg.status == 10:
            message = "PATH_PLANNER_SERVICE_FAILED"
            self.stop_experiment_pub.publish(Empty())
        if msg.status == 11:    
            message = "PATH_PLANNER_FAILED"
            self.stop_experiment_pub.publish(Empty())
        if msg.status == 12:
            message = "PATH_PLANNER_REPOSITIONING_FAILED"
            self.stop_experiment_pub.publish(Empty())
        if msg.status == 13:
            message = "POLYGONCONSTRAINT_SERVICE_SUCCESS"
        if msg.status == 14:    
            message = "POLYGONCONSTRAINT_SERVICE_FAILED"
            self.stop_experiment_pub.publish(Empty())
        if msg.status == 15:
            message = "SMOOTHING_SERVICE_SUCCESS"
        if msg.status == 16:
            message = "SMOOTHING_SERVICE_FAILED"
            self.stop_experiment_pub.publish(Empty())
        if msg.status == 17:
            message = "SMOOTHING_FAILED"
            self.stop_experiment_pub.publish(Empty())
        if msg.status == 18:
            message = "DELTATVEC_SERVICE_SUCCESS"
        if msg.status == 19:
            message = "DELTATVEC_SERVICE_FAILURE"
            self.stop_experiment_pub.publish(Empty())
        if msg.status == 20:    
            message = "DELTATVEC_CONSTRAINT_FAILURE"
            self.stop_experiment_pub.publish(Empty())

if __name__ == '__main__':
    rospy.init_node("iliad_start_stop_experiments_node")#, log_level=rospy.DEBUG)
    try:
        sse = start_stop_experiments()
    except rospy.ROSInterruptException:
        rospy.loginfo("Error -exception raised") 