#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Call service orunav_msgs/ExecuteTask  at  /robot4/execute_task 
to abort task!

"""


import rospy
from orunav_msgs.srv import  ExecuteTask, ExecuteTaskRequest, ExecuteTaskResponse
from orunav_msgs.msg import Operation
from orunav_msgs.msg import Task
from orunav_msgs.msg import ControllerReport
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class TaskCancellator():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params and atributes
        self.prev_time = rospy.Time.now()
        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [" + rospy.get_name() + "] Started.")



    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)

        self.execute_task_srv_name = rospy.get_param(
            '~execute_task_srv_name', '/robot' + str(self.robot_id) + '/execute_task')

        self.curr_task_topic_name  = rospy.get_param(
            '~task_topic_name', '/robot' + str(self.robot_id) + '/control/task')

        self.reports_topic_name = rospy.get_param(
            '~reports_topic', '/robot' + str(self.robot_id) + '/control/controller/reports')


    def initROS(self):
        # topic publishers
        
        # service clients
        rospy.loginfo("Node [" + rospy.get_name() + "] waiting for service [" +self.execute_task_srv_name +"] to be available")
        rospy.wait_for_service(self.execute_task_srv_name)
        try:
            self.execute_task_srv_px = rospy.ServiceProxy(self.execute_task_srv_name, ExecuteTask)
        except rospy.ServiceException as e:
            rospy.logerr("Node [" + rospy.get_name() + "] Could not create srv proxy: " + str(e))
        
        # service servers
        
        # topic subscribers and other listeners
        rospy.Subscriber(self.curr_task_topic_name, Task, self.curr_task_callback, queue_size=1)
        rospy.Subscriber(self.reports_topic_name, ControllerReport, self.reports_callback, queue_size=1)

    def curr_task_callback(self,msg):
        self.active_task = msg
        rospy.loginfo("Node [" + rospy.get_name() + "] Detected a new active Task!")
        rospy.sleep(10)
        rospy.loginfo("Node [" + rospy.get_name() + "] Trying to cancel it!")
        self.abort_task()    


    # we use robot reports to know robot position and speed
    def reports_callback(self, msg):
            nowTime = rospy.Time.now()
            try:
                self.prev_state = self.state
            except AttributeError as ae:
                self.prev_state = msg.state

            self.state = msg.state
            self.robot_x = msg.state.position_x
            self.robot_y = msg.state.position_y
            self.robot_h = msg.state.orientation_angle
            self.robot_v = 0
            self.robot_w = 0


            inc_t = (nowTime - self.prev_time).to_sec()
            inc_x = self.state.position_x - self.prev_state.position_x
            inc_y = self.state.position_y - self.prev_state.position_y
            inc_r = np.sqrt((inc_x ** 2) + (inc_y ** 2))
            self.robot_v = inc_r/inc_t
            inc_h = self.state.orientation_angle - self.prev_state.orientation_angle
            self.robot_w = inc_h/inc_t

            rospy.logdebug_throttle(5, "Node [" + rospy.get_name() + "] " +
                                    "Robot status: Pose ( " +
                                    str(self.state.position_x) + ", " + str(self.state.position_y) + ", " +
                                    str(self.state.orientation_angle*180.0/np.pi) + " deg), " +
                                    "Speed ( " +
                                    str(self.robot_v) + " m/sec, " +
                                    str(self.robot_w * 180.0/np.pi) + " deg/sec) "
                                    )
            self.prev_time = nowTime


    def findClosestInPath(self):
        # Not the smartest search ...
        dist = 1e10
        ind = -1
        for i,posSteer in enumerate(self.active_task.path.path):
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
        

    def get_rotation (self,orientation_q):
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw
    
    def abort_task(self):
        index_offset = 3
        try:
            req = ExecuteTaskRequest()
            #req.task = self.active_task
            req.task = self.active_task
            n_points_orig = len(self.active_task.path.path)

            index_mod_start = self.findClosestInPath()
            index_mod_end =  n_points_orig - 1 - index_offset

            n_mod_points = index_mod_end - index_mod_start + 1

            if (n_mod_points>0):
                for i in range(index_mod_start,index_mod_end+1):
                    # curve with no offset at 0-end
                    #offset = (4.0 * (n_mod_points-i) * (i)/ (n_mod_points * n_mod_points ) )
                    offset = (4.0 * (index_mod_end-i) * (i-index_mod_start)/ ((n_mod_points-1) * (n_mod_points-1) ) )
                    req.task.path.path[i].pose.position.x = req.task.path.path[i].pose.position.x + 1.0 * offset
                    req.task.path.path[i].pose.position.y = req.task.path.path[i].pose.position.y + 1.0 * offset
                    #rospy.loginfo("Node [" + rospy.get_name() + "] p[" + str(i)+  " offset == " + str(offset) )

                req.task.abort = False
                req.task.update = True       

                rospy.loginfo("Node [" + rospy.get_name() + "] Selected indexes are: [" + str(index_mod_start) + ":" 
                                                                                        + str(index_mod_end)   + "] of " 
                                                                                        + str(len(self.active_task.path.path)) )   
                rospy.loginfo("Node [" + rospy.get_name() + "] " +
                                        "Robot at: Pose ( " +
                                        str(self.state.position_x) + ", " + str(self.state.position_y) + ", " +
                                        str(self.state.orientation_angle*180.0/np.pi) + " deg) ")
                rospy.loginfo("Node [" + rospy.get_name() + "] " +
                                        "Goal at: Pose ( " +
                                        str(req.task.path.target_goal.pose.position.x) + ", " + str(req.task.path.target_goal.pose.position.y) + ", " +
                                        str(self.get_rotation(req.task.path.target_goal.pose.orientation)*180.0/np.pi) + " deg) ")

                try:
                    ans = self.execute_task_srv_px.call(req)
                    rospy.loginfo("Node [" + rospy.get_name() + "] result was: "+str(ans))   
                    rospy.signal_shutdown("Shutdown successful")
                except rospy.ServiceException as e:
                    rospy.logerr("Node [" + rospy.get_name() + "] Could not call service: " + str(e))
            else:
                rospy.logerr("Node [" + rospy.get_name() + "] Not enough points to cancel " )
        except AttributeError as ae:
            rospy.logwarn("Node [" + rospy.get_name() + "] Does not have enough info to abort task: " + str(ae))
        

# Main function.
if __name__ == '__main__':
    rospy.init_node('TaskCancel', log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = TaskCancellator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
