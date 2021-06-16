#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

"""


import rospy
from std_msgs.msg import Float64, Float64MultiArray, String
from orunav_msgs.srv import  Trigger
from orunav_msgs.msg import Task, ComputeTaskStatus,ReplanStatus
from std_srvs.srv import Empty

import numpy as np
from threading import Lock

class object_aware_assessment():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params and atributes
        self.lock = Lock()
        self.curr_cost = None
        self.situation = None
        self.v = self.v_max 
        self.w = self.w_max 
        self.v_rev = self.v_rev_max
        self.w_rev = self.w_rev_max

        self.old_v = None
        self.old_w = None
        self.old_v_rev = None
        self.old_w_rev = None

        self.replan_needed = 0
        self.replan_finished = False
        self.replan_triggered = False
        self.replan_wait_start = 0

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [" + rospy.get_name() + "] Started.")

    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)
        
        # current robot task
        #NOT SURE! self.curr_task_topic_name  = rospy.get_param('~task_topic_name', '/robot' + str(self.robot_id) + '/control/task')

        # human tracking trackedpersons (SPENCER) topic
        #NOT SURE! self.human_tracking_topic_name = rospy.get_param('~human_tracking_topic_name', '/robot'+str(self.robot_id)+'/qsr/people_tracker/positions')

        #current robot position from tf tree 
        #NOT SURE! self.robot_pose_topic_name = rospy.get_param('~robot_pose_topic_name', '/robot' + str(self.robot_id) + '/robot_poseST')
        
        # current task cost
        self.curr_cost_topic_name = rospy.get_param('~curr_cost_topic_name', '/robot' + str(self.robot_id) + '/qsr/cost')

        # where do we publish speed constraints: v and w
        self.velocity_constraints_topic = rospy.get_param('~velocity_constraints_topic', '/robot'+str(self.robot_id)+'/velocity_constraints')

        # We get these initial values from vehicle execution node
        self.ven_name = rospy.get_param('~ven_name', 'orunav_vehicle_execution_node')
        self.ven_name = '/robot'+ str(self.robot_id) +'/' + self.ven_name

        # this max forward speed
        self.v_max = rospy.get_param(self.ven_name + '/max_vel', 0.1)
        # this max left turning speed
        self.w_max = rospy.get_param(self.ven_name + '/max_rotational_vel', 0.1 )
        # this max backward speed
        self.v_rev_max = rospy.get_param(self.ven_name + '/max_vel', self.v_max)
        # this max right turning speed
        self.w_rev_max = rospy.get_param(self.ven_name + '/max_rotational_vel', self.w_max)
        
        # this is the minimum speed to prevent mpc to go bananas
        self.v_min = 0.01
        self.w_min = 0.01 

        # topic to read to ensure that the path has been computed
        self.time_needed_before_replan = rospy.get_param('~time_needed_before_replan',10)
        self.clear_costmap_service_name = rospy.get_param('~clear_costmap_service_name','/object_costmap_node/clear_costmaps')


    def initROS(self):
        # topic publishers
        self.velocity_constraints_pub = rospy.Publisher(self.velocity_constraints_topic, Float64MultiArray, queue_size=1)

        # topic subscribers and other listeners
        rospy.Subscriber(self.curr_cost_topic_name, Float64, self.curr_cost_callback, queue_size=1)
        rospy.Subscriber('/robot' + str(self.robot_id) + '/compute_task/status', ComputeTaskStatus, self.compute_task_callback, queue_size=1)
        rospy.Subscriber("/coordinator/replan/status", ReplanStatus, self.replan_status_callback, queue_size=1)

        # service servers
        rospy.loginfo("Waiting for the /coordinator/replan service to be available" )
        rospy.wait_for_service('/coordinator/replan')
        self.replan_service_client = rospy.ServiceProxy('/coordinator/replan',Trigger)

        rospy.loginfo("Waiting for the "+ self.clear_costmap_service_name+" service to be available" )
        rospy.wait_for_service(self.clear_costmap_service_name)
        self.clear_costmap_client = rospy.ServiceProxy(self.clear_costmap_service_name,Empty)

        #timers
        self.check_for_replan_timer = rospy.Timer(rospy.Duration(0.5),self.check_for_replan)


    def curr_cost_callback(self, msg):
        update = False
        with self.lock:         
            update = (self.curr_cost != msg.data)
            self.curr_cost = msg.data
        if update:
            self.assesst()

            
    def assesst(self):
        with self.lock:       
            if (self.curr_cost) == 0:
                self.v = self.v_max
                self.w = self.w_max
                self.v_rev = self.v_rev_max
                self.w_rev = self.w_rev_max

                self.replan_needed = 0

            else:
                self.v = self.v_min
                self.w = self.w_min
                self.v_rev = self.v_min
                self.w_rev = self.w_min                

                self.replan_needed = 1


            self.sendNewConstraints()



    def sendNewConstraints(self):
        #only update constraints if they are different
        if self.old_v != self.v and self.old_w != self.w and self.old_v_rev != self.v_rev and self.old_w_rev != self.w_rev:
            self.old_v = self.v
            self.old_w = self.w
            self.old_v_rev = self.v_rev
            self.old_w_rev = self.w_rev
            msg = Float64MultiArray()
            msg.data.append(self.v)
            msg.data.append(self.w)
            msg.data.append(self.v_rev)
            msg.data.append(self.w_rev)
            self.velocity_constraints_pub.publish(msg)
            rospy.loginfo("Node [" + rospy.get_name() + "] " +
                                "New speed Constraints sent : V ( -" +
                                str(self.v_rev) + ", " + str(self.v) + " m/s), " +
                                "W ( -" +
                                str(self.w_rev * 180.0/np.pi) + ", " + str(self.w * 180.0/np.pi) + " deg/sec) "
                                )

    def compute_task_callback(self,msg): #this is just for debugging purposes
        if self.replan_triggered == True:
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
            if msg.status == 7:
                message = "GEOFENCE_CALL_SUCCESS"
            if msg.status == 8: 
                message = "GEOFENCE_CALL_FAILURE"
            if msg.status == 9:
                message = "PATH_PLANNER_SERVICE_SUCCESS"
            if msg.status == 10:
                message = "PATH_PLANNER_SERVICE_FAILED"
            if msg.status == 11:    
                message = "PATH_PLANNER_FAILED"
            if msg.status == 12:
                message = "PATH_PLANNER_REPOSITIONING_FAILED"
            if msg.status == 13:
                message = "POLYGONCONSTRAINT_SERVICE_SUCCESS"
            if msg.status == 14:    
                message = "POLYGONCONSTRAINT_SERVICE_FAILED"
            if msg.status == 15:
                message = "SMOOTHING_SERVICE_SUCCESS"
            if msg.status == 16:
                message = "SMOOTHING_SERVICE_FAILED"
            if msg.status == 17:
                message = "SMOOTHING_FAILED"
            if msg.status == 18:
                message = "DELTATVEC_SERVICE_SUCCESS"
            if msg.status == 19:
                message = "DELTATVEC_SERVICE_FAILURE"
            if msg.status == 20:    
                message = "DELTATVEC_CONSTRAINT_FAILURE"

            rospy.loginfo(message)

    def replan_status_callback(self,msg):
        if self.replan_triggered == True:
            if msg.robot_id == self.robot_id:
                if msg.status == 0:
                    message = "REPLAN_STARTED"
                if msg.status == 1:
                    message = "REPLAN_SUCCESS"
                    self.replan_finished = True
                if msg.status == 2: 
                    message = "REPLAN_FAILURE"
                    self.replan_finished = True

                rospy.loginfo(message)

    def check_for_replan(self,timer):
        if self.replan_triggered == True and self.replan_finished == False:
            rospy.loginfo("Waiting for replan to be computed..." )
            self.replan_wait_start = 0
        else:
            self.replan_triggered = False
            self.replan_finished = False
            if self.replan_needed == 1 and self.replan_wait_start ==0:
                self.replan_wait_start = 1
                self.start_time = rospy.get_time()
                rospy.loginfo("Object on the way. Doing a replan in ..." )

            if self.replan_needed == 1 and self.replan_wait_start == 1:
                if rospy.get_time() - self.start_time > self.time_needed_before_replan:
                    rospy.loginfo("Replan triggered" )
                    #clear the costmap to remove old objects not seen anymore
                    clear_success = self.clear_costmap_client()
                    if clear_success == False:
                        rospy.loginfo("Clear costmap failed" )
                        self.start_time = rospy.get_time()
                    else:
                        rospy.loginfo("Clear costmap succesful" )
                        self.replan_triggered = True

                    #ask the coordinator here to do a replan
                    success = self.replan_service_client(self.robot_id)
                    if success == False:
                        rospy.loginfo("Replan call failed" )
                        self.start_time = rospy.get_time()
                    else:
                        rospy.loginfo("Replan call succesful" )
                        self.replan_triggered = True

                else:
                    rospy.loginfo(self.time_needed_before_replan - (rospy.get_time() - self.start_time))

            if self.replan_needed == 0:
                self.replan_wait_start = 0


# Main function.
if __name__ == '__main__':
    rospy.init_node('object_aware_plan_assessment_node', log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = object_aware_assessment()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
