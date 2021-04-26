#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

"""


import rospy
from std_msgs.msg import Float64, Float64MultiArray, Empty, String
from orunav_msgs.srv import  RePlan

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

        self.replan_needed = 0

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
        
        # current human robot interaction from Laurences node
        self.situation_topic_name = rospy.get_param('~situation_topic_name', '/robot' + str(self.robot_id) + '/qsr/situation_predictions')

        # current task cost
        self.curr_cost_topic_name = rospy.get_param('~curr_cost_topic_name', '/robot' + str(self.robot_id) + '/qsr/cost')

        # where do we publish speed constraints: v and w
        self.velocity_constraints_topic = rospy.get_param('~velocity_constraints_topic', '/robot'+str(self.robot_id)+'/velocity_constraints')

        # tells the coordinator to stop and replan to current goal
        self.trigger_topic_name = rospy.get_param('~trigger_topic_name', '/robot' + str(self.robot_id) + '/qsr/replan')

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


    def initROS(self):
        # topic publishers
        self.velocity_constraints_pub = rospy.Publisher(self.velocity_constraints_topic, Float64MultiArray, queue_size=1)
        self.trigger_topic_pub = rospy.Publisher(self.trigger_topic_name, Empty, queue_size=1)

        # service clients
        # none here

        # tf buffers
        # none here


        # topic subscribers and other listeners
        rospy.Subscriber(self.curr_cost_topic_name, Float64, self.curr_cost_callback, queue_size=1)
        rospy.Subscriber(self.situation_topic_name, String, self.situation_callback, queue_size=1)

        # service servers
        rospy.loginfo("Waiting for the /coordinator/replan service to be available" )
        rospy.wait_for_service('/coordinator/replan')
        self.replan_service_client = rospy.ServiceProxy('/coordinator/replan',RePlan)

        #timers
        self.check_for_replan_timer = rospy.Timer(rospy.Duration(0.5),self.check_for_replan)
            

    def curr_cost_callback(self, msg):
        update = False
        with self.lock:         
            update = (self.curr_cost != msg.data)
            self.curr_cost = msg.data
        if update:
            self.assesst()

    def situation_callback(self, msg):
        update = False
        with self.lock:         
            update = (self.situation != msg.data)
            self.situation = msg.data.upper()
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
        msg = Float64MultiArray()
        msg.data.append(self.v)
        msg.data.append(self.w)
        msg.data.append(self.v_rev)
        msg.data.append(self.w_rev)
        self.velocity_constraints_pub.publish(msg)
        rospy.loginfo_throttle(3,"Node [" + rospy.get_name() + "] " +
                            "New speed Constraints sent : V ( -" +
                            str(self.v_rev) + ", " + str(self.v) + " m/s), " +
                            "W ( -" +
                            str(self.w_rev * 180.0/np.pi) + ", " + str(self.w * 180.0/np.pi) + " deg/sec) "
                            )

    def check_for_replan(self,timer):
        if self.replan_needed == 1 and self.replan_wait_start ==0:
            self.replan_wait_start = 1
            self.start_time = rospy.get_time()
            rospy.loginfo("Object on the way. Doing a replan in ..." )

        if self.replan_needed == 1 and self.replan_wait_start == 1:
            if rospy.get_time() - self.start_time > 15:
                rospy.loginfo("Replan triggered" )

                #ask the coordinator here to do a replan
                success = self.replan_service_client(self.robot_id)
                if success:
                    rospy.loginfo("Replan succesful" )
                    self.start_time = rospy.get_time()
                else:
                    rospy.loginfo("Replan failed" )
                    self.start_time = rospy.get_time()



            else:
                rospy.loginfo(15 - (rospy.get_time() - self.start_time))

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
