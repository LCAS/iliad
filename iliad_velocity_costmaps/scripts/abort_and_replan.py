#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Periodically calls service orunav_msgs/ComputeTask  at  /robotX/compute_task 
to have an alternative task stored.

Upon request on empty type topic:
Calls service orunav_msgs/UpdateTask  at  /coordinator/update_task 
to update task with alternative task.
"""


import rospy
from orunav_msgs.srv import  UpdateTask, UpdateTaskRequest, UpdateTaskResponse
from orunav_msgs.srv import  ComputeTask, ComputeTaskRequest, ComputeTaskResponse
from orunav_msgs.msg import Operation
from orunav_msgs.msg import Task
from orunav_msgs.msg import ControllerReport
from std_msgs.msg import Empty
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from scipy.interpolate import UnivariateSpline

class TaskReplanner():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params and atributes
        self.state = None
        self.controller_status = None
        self.active_task = None
        self.goalPS = None

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [" + rospy.get_name() + "] Started.")



    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)

        self.update_task_srv_name = rospy.get_param(
            '~update_task_srv_name', '/coordinator/update_task')

        self.curr_task_topic_name  = rospy.get_param(
            '~task_topic_name', '/robot' + str(self.robot_id) + '/control/task')

        self.trigger_topic_name  = rospy.get_param(
            '~trigger_topic_name', '/robot' + str(self.robot_id) + '/update_task')

        self.reports_topic_name = rospy.get_param(
            '~reports_topic_name', '/robot' + str(self.robot_id) + '/control/controller/reports')

        self.goal_topic_name = rospy.get_param(
            '~goal_topic_name', '/robot' + str(self.robot_id) + '/goal')

        self.brake_topic_name = rospy.get_param(
            '~brake_topic_name', '/robot'+str(self.robot_id)+'/brake')       

        # task frame id
        self.goal_frame_id = rospy.get_param(
            '~goal_frame_id', 'map_laser2d')                 

    def initROS(self):
        # topic publishers
        self.brake_topic_pub = rospy.Publisher(self.brake_topic_name, Int32, queue_size=1)
        self.goal_topic_pub = rospy.Publisher(self.goal_topic_name, PoseStamped, queue_size=1)
        
        # service clients
        rospy.loginfo("Node [" + rospy.get_name() + "] waiting for service [" +self.update_task_srv_name +"] to be available")
        rospy.wait_for_service(self.update_task_srv_name)
        try:
            self.update_task_srv_px = rospy.ServiceProxy(self.update_task_srv_name, UpdateTask)
        except rospy.ServiceException as e:
            rospy.logerr("Node [" + rospy.get_name() + "] Could not create srv proxy: " + str(e))

        # service servers
        
        # topic subscribers and other listeners
        rospy.Subscriber(self.curr_task_topic_name, Task, self.curr_task_callback, queue_size=1)
        rospy.Subscriber(self.reports_topic_name, ControllerReport, self.reports_callback, queue_size=1)
        rospy.Subscriber(self.trigger_topic_name, Empty, self.trigger_callback, queue_size=1)

    def curr_task_callback(self,msg):
        self.active_task = msg
        self.active_task_time = rospy.Time.now()  
        rospy.loginfo("Node [" + rospy.get_name() + "] Detected a new active Task!")

    def trigger_callback(self,msg):
        rospy.loginfo("Node [" + rospy.get_name() + "] Trying to replan it!")
        self.replan_task()    

    def stop(self,isStopping):
        msg = Int32()
        msg.data = 2 # RECOVER
        if isStopping:
            msg.data = 0 # STOP
        self.brake_topic_pub.publish(msg)
            
    # we use robot reports to know robot position and state
    def reports_callback(self, msg):
            self.state = msg.state
            self.controller_status = msg.status

            if (self.controller_status == 1) and (self.goalPS!=None):
                rospy.loginfo("Node [" + rospy.get_name() + "] Goal to be send in some seconds ...")   
                rospy.sleep(5.1)
                rospy.loginfo("Node [" + rospy.get_name() + "] Asking for a new one ......................")      
                self.goalPS.header.stamp = rospy.Time.now()   
                self.goalPS.header.frame_id = "comemeElPapo"
                self.goal_topic_pub.publish(self.goalPS)
                self.goalPS=None
                rospy.loginfo("Node [" + rospy.get_name() + "] Done")   



    def findClosestInPath(self,path):
        # Not the smartest search ...
        dist = 1e10
        ind = -1
        for i,posSteer in enumerate(path):
            posit_i = posSteer.pose.position
            dist_i = self.getDist(posit_i,self.state)
            if (dist>dist_i):
                ind = i
                dist = dist_i
        return ind

    def getDist(self, position_i, state_i ):
        # I really don't like substracting points assuming same refernce frame ...
        dx = state_i.position_x - position_i.x
        dy = state_i.position_y - position_i.y
        # dt = state_i.orientation_angle - 
        dist = np.sqrt(dx*dx + dy*dy)   
        return dist     

    def abort_task(self):        
        if ((not self.state == None) and (not self.active_task == None)):
            req = UpdateTaskRequest()
            req.task = self.active_task
        
            # where are we?
            curr_index = self.findClosestInPath(self.active_task.path.path)
            index_offset = 5
            end_index = min(curr_index+index_offset,len(self.active_task.path.path)-1)
            req.task.path.path = req.task.path.path[:end_index]
            req.task.target.goal = req.task.path.path[-1]

            req.task.update = True
            req.task.abort = True
           
            try:
                ans = self.update_task_srv_px.call(req)
                if ans.result != 0:
                    rospy.logerr("Node [" + rospy.get_name() + "] Abort task result was FAILED: "+str(ans.result))   
                else:
                    rospy.loginfo("Node [" + rospy.get_name() + "] Abort task result was SUCCESS: "+str(ans.result))   
            except rospy.ServiceException as e:
                rospy.logerr("Node [" + rospy.get_name() + "] Could not call service: [" + self.update_task_srv_name + "] to Abort task:" + str(e))

    def replan_task(self):        
        if ((not self.state == None) and (not self.active_task == None)):
            # here in case active task gets smashed by abort task
            goal = self.active_task.target.goal
            rospy.loginfo("Node [" + rospy.get_name() + "] Stopping current plan ......................")   
            self.abort_task()
            self.goalPS =self.poseSteer2PoseSt(goal,self.goal_frame_id)

    def poseSteer2PoseSt(self, poseSteer,frame_id):
        pose_in = PoseStamped()
        pose_in.pose = poseSteer.pose
        pose_in.header.frame_id = frame_id
        return pose_in

# Main function.
if __name__ == '__main__':
    rospy.init_node('task_replan_node', log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = TaskReplanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
