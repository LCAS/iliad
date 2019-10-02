#!/usr/bin/env python

from geometry_msgs.msg import Vector3
'''



'''

import rospy 
import numpy as np
from orunav_msgs.msg import ControllerState, ControllerReport, Task
from orunav_msgs.srv import  ExecuteTask, ExecuteTaskRequest, ExecuteTaskResponse
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
import tf2_geometry_msgs
import tf2_ros


class TaskManipulationNode():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params and atributes
        self.first_time =True

        # states from mpc node
        self.state = None
        self.robotPose = Pose()
        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [" + rospy.get_name() + "] entering spin...")

        rospy.spin()

    def find_param(self, param_name, rid, default_val):
        ans = default_val
        allParams = rospy.get_param_names()
        for pi in allParams:
            if (pi[1:7] == ('robot'+str(rid))):
                if param_name in pi:
                    ans = rospy.get_param(pi, ans)
                    break

        return ans

    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)

        # minimum number of points in path to consider adjusting
        self.min_path_points = rospy.get_param('~min_path_points', 4)

        self.reports_topic = rospy.get_param(
            '~reports_topic', '/robot' + str(self.robot_id) + '/control/controller/reports')

        self.task_topic = rospy.get_param(
            '~task_topic', '/robot' + str(self.robot_id) + '/control/controller/task')

        self.execute_task_srv_name = rospy.get_param(
            '~execute_task_srv_name', '/robot' + str(self.robot_id) + '/execute_task')

        # robot frame id
        self.base_frame_id = rospy.get_param(
            '~base_frame', '/robot'+str(self.robot_id)+'/base_link')
        if (self.base_frame_id[0] == '/'):
            self.base_frame_id = self.base_frame_id[1:]

        # tranform tf_timeout
        timeout = rospy.get_param('~tf_timeout', 2)
        self.tf_timeout = rospy.Duration(timeout)

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
        rospy.Subscriber(self.reports_topic, ControllerReport,
                         self.reports_callback, queue_size=1)

        rospy.Subscriber(self.task_topic, Task,
                         self.task_callback, queue_size=1)                         

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)


    # .............................................................................................................

    # we use robot reports to know robot position
    def reports_callback(self, report_msg):
        self.state = report_msg.state

        self.robotPose.position.x = self.state.position_x
        self.robotPose.position.y = self.state.position_y
        self.robotPose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.state.orientation_angle) )
        
        #rospy.loginfo_throttle(1,".")

        

    # what is the robot up to...
    def task_callback(self, task_msg):
        self.task = task_msg
        rospy.loginfo(" ..................................................." )
        rospy.loginfo("Node [" + rospy.get_name() + "] Got New Task with ["+str(len(self.task.path.path))+"] points")
        rospy.loginfo(" ...................................................\n\n" )
        
        if self.first_time:
            rospy.Timer(rospy.Duration(1.0), self.altertask_tcb, True)
            

    def altertask_tcb(self,event):
        # find in which point of the path robot is
        max_i = len(self.task.path.path)
        min_dist = np.inf
        min_i = max_i
        

        for i,ps in enumerate(self.task.path.path):
            d = self.poseDist(ps.pose, self.robotPose)
            if d<min_dist:
                min_dist=d
                min_i = i

        enough_points = (max_i-min_i)>(self.min_path_points)

        rospy.loginfo(" ..................................................." )
        if not enough_points:        
            rospy.logwarn("Node [" + rospy.get_name() + "] Not enough remaining points to alter: "+str(max_i-min_i))
        elif (min_dist>1.0):
            rospy.logwarn("Node [" + rospy.get_name() + "] Too far to consider path modification: "+str(min_dist))            
        else:
            self.first_time = False
            rospy.loginfo(" ..................................................." )
            rospy.loginfo("Node [" + rospy.get_name() + "] ALTERING PATH!!! ")
            rospy.loginfo("Node [" + rospy.get_name() + "] robot-path min distance is: "+str(min_dist))            
            rospy.loginfo("Node [" + rospy.get_name() + "] at point ["+str(min_i)+ "]" )
            rospy.loginfo(" ...................................................\n" )
            rospy.loginfo("Node [" + rospy.get_name() + "] Cloning remaining path" )
            newTask = Task()
            newTask = self.task # hope this copies task data
            newTask.target.start.pose = self.robotPose
            newTask.target.start.steering = self.state.steering_angle
            # not used???
            #newTask.path.target_start =  newTask.target.start
            newTask.path.path=self.task.path.path[max(min_i-1,0):]
            k0 = len(self.task.path.path)
            k = len(newTask.path.path)
            newTask.path.path[0]=newTask.target.start
            newTask.update = True
            # TODO  newTask.criticalPoint needs to be changed!!!!
            if newTask.criticalPoint>-1:
                newTask.criticalPoint= newTask.criticalPoint - (k0-k)

            rospy.loginfo(" ...................................................\n" )
            rospy.loginfo("Node [" + rospy.get_name() + "] Deviated path:" )

            

            newTask.path.path[k/2].pose.position.x = newTask.path.path[k/2].pose.position.x + 1.5
            newTask.path.path[k/2].pose.position.y = newTask.path.path[k/2].pose.position.y + 0

            # Let's alter all remaining path points but start and end
            # m = k -1
            # for i in range(1,m):
            #     newTask.path.path[i].pose.position.x = newTask.path.path[i].pose.position.x + (0.005*(i)*(i-m))
            #     newTask.path.path[i].pose.position.y = newTask.path.path[i].pose.position.y + (0.005*(i)*(i-m))



            rospy.loginfo(" ..................................................." )
            rospy.loginfo("Node [" + rospy.get_name() + "] Updating task ")   
            rospy.loginfo("Node [" + rospy.get_name() + "] New path len is "+str(k))   

            req = ExecuteTaskRequest()
            req.task = newTask
            try:
                ans = self.execute_task_srv_px.call(req)
            except rospy.ServiceException as e:
                rospy.logerr("Node [" + rospy.get_name() + "] Could not call service: " + str(e))
            
            rospy.loginfo("Node [" + rospy.get_name() + "] result was: "+str(ans))   
            


    def poseDist(self,poseA,poseB):
        d = np.sqrt(  (  (poseA.position.x - poseB.position.x) ** 2) + ((poseA.position.y - poseB.position.y) ** 2) + ((poseA.position.z - poseB.position.z) ** 2) )
        return d

# Main function.
if __name__ == '__main__':
    rospy.init_node('TaskConstraintsNode')#, log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = TaskManipulationNode()
    except rospy.ROSInterruptException:
        pass
