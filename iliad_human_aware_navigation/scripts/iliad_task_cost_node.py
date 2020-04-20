#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Subscribes to constraints map and Task and publishes current task cost.
"""


import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from message_filters import Subscriber
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseStamped, Quaternion

from bayes_people_tracker.msg import PeopleTracker
from orunav_msgs.msg import ControllerReport, Task

import numpy as np
from threading import Lock


class TaskCostEvaluator(object):
    def __init__(self, name):
        rospy.loginfo("["+rospy.get_name()+"] " + "Starting ... ")

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # other params
        self.lock = Lock()
        self.active_task = None
        self.active_costmap = None
        self.state = None
        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()
        
        rospy.loginfo("["+rospy.get_name()+"] " + "... all done.")

    def initROS(self):
        # publishers
        self.path_occ_grid_pub = rospy.Publisher(self.path_occ_grid_topic_name, OccupancyGrid, queue_size=1)
        self.curr_cost_topic_pub = rospy.Publisher(self.curr_cost_topic_name, Float64, queue_size=1)
        # service clients
        
        # ... none here

        # subscribers and listeners
        self.listenerBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.listenerBuffer)

        rospy.Subscriber(self.task_topic_name, Task, self.task_callback, queue_size=1)                
        rospy.Subscriber(self.costmap_topic_name, OccupancyGrid, self.costmap_callback, queue_size=1)                
        rospy.Subscriber(self.reports_topic_name, ControllerReport, self.reports_callback, queue_size=1)

        # service servers

        # ... none here

    # .............................................................................................................

    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)

        # Costmap topic
        self.costmap_topic_name = rospy.get_param(
            '~costmap_topic_name', '/robot'+str(self.robot_id)+'/qsr/constraints_costmap')

        # task topic
        self.task_topic_name = rospy.get_param(
            '~task_topic_name', '/robot' + str(self.robot_id) + '/control/task')

        # task frame id
        self.task_frame_id = rospy.get_param(
            '~task_frame_id', '/world')

        # path costmap topic
        self.path_occ_grid_topic_name = rospy.get_param(
            '~path_occ_grid_topic_name', '/robot'+str(self.robot_id)+'/qsr/constraints_costmap_path')

        # reports topic name
        self.reports_topic_name = rospy.get_param(
            '~reports_topic_name', '/robot' + str(self.robot_id) + '/control/controller/reports')

        # current cost topic name
        self.curr_cost_topic_name = rospy.get_param(
            '~curr_cost_topic_name', '/robot' + str(self.robot_id) + '/qsr/cost')

    # we use robot reports to know robot position
    def reports_callback(self, msg):
            self.state = msg.state

    def task_callback(self,msg):
        self.active_task = msg
        rospy.loginfo("Node [" + rospy.get_name() + "] Detected a new active Task!")
        self.updateCost()

    def costmap_callback(self,msg):
        self.active_costmap = msg
        height = self.active_costmap.info.height
        width = self.active_costmap.info.width        
        
        # as a 2d grid access is easier ...
        self.active_costmap_local_grid = np.array(self.active_costmap.data).reshape(height,width).T

        rospy.logdebug("Node [" + rospy.get_name() + "] Detected a new costmap!")
        self.createPathCostGrid()
        self.updateCost()

    def createPathCostGrid(self):
        o = OccupancyGrid()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = self.active_costmap.header.frame_id
        o.info.resolution = self.active_costmap.info.resolution
        o.info.height = self.active_costmap.info.height
        o.info.width = self.active_costmap.info.width
        o.info.origin = Pose()

        # Translate to be centered under costmap_frame_id
        o.info.origin.position.x -= (o.info.width/2.0) * o.info.resolution 
        o.info.origin.position.y -= (o.info.height/2.0)  * o.info.resolution
        # as a 2d grid access is easier ...
        self.path_occ_grid_local_grid = np.full((o.info.width, o.info.height), 0)

        # not really necesary till publishing...
        o.data = self.path_occ_grid_local_grid.flatten(order='C')
     
        self.path_occ_grid = o
        
    def updateCost(self):
        with self.lock: 
            if ((not self.active_task == None) and (not self.active_costmap == None) and (not self.state == None)):
                newPosePath = []
                newCellPath = []
                newCostPath = []
                pendingPath = self.active_task.path.path
                # get part of the path still to be covered
                i_path      = self.findClosestInPath(pendingPath,self.state)
                pendingPath = pendingPath[i_path:]
                # get costs of this pending path
                for posSteer in pendingPath:
                    localPoseSt = self.poseSteer2costmapPose(posSteer)
                    (px, py, ci, cj, val)  = self.getCostmapValue(self.active_costmap, self.active_costmap_local_grid, localPoseSt.pose.position.x , localPoseSt.pose.position.y)
                    self.setCostmapValue(self.path_occ_grid, self.path_occ_grid_local_grid, px, py, val)
                    #if not (ci,cj) in newCellPath:
                    newPosePath.append((px, py))
                    newCellPath.append((ci,cj))
                    newCostPath.append(val)  

                self.path_occ_grid.data =  self.path_occ_grid_local_grid.T.flatten(order='C')            
                self.path_occ_grid_pub.publish(self.path_occ_grid)
                # and this to test cost calculus
                X, Y = zip(*newPosePath)
                X = np.array(X)
                Y = np.array(Y)
                C = np.array(newCostPath)
                dx = np.diff(X)
                dy = np.diff(Y)
                dr = np.sqrt(dx*dx+dy*dy)
                dr0 = np.concatenate((dr, [0]), axis=0)
                dr1 = np.concatenate(([0], dr), axis=0)
                dr2 = dr0/2 + dr1/2
                w = dr2.sum()
                cost = C.dot(dr2)
                if w>0:
                    cost = cost/dr2.sum()
                self.curr_cost_topic_pub.publish(cost)
                # mfc: using this metric, path cost can increase even if costmap does not change, just because the human happens to be at the end of the path                
                #rospy.loginfo("["+rospy.get_name()+"] " + "Current map cost is: " + str(cost))


    def findClosestInPath(self,poseStpath,robotState):
        # Not the smartest search ...
        dist = 1e10
        ind = -1
        for i,posSteer in enumerate(poseStpath):
            posit_i = posSteer.pose.position
            dist_i = self.getDist(posit_i,robotState)
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

    def poseSteer2costmapPose(self, poseSteer):
        poseSt_out = None
        now = rospy.Time.now()

        pose_in = PoseStamped()
        pose_in.pose = poseSteer.pose
        pose_in.header.frame_id = self.task_frame_id
        pose_in.header.stamp = now
        
        # in tf2, frames do not have the initial slash
        if (pose_in.header.frame_id[0] == '/'):
            pose_in.header.frame_id = pose_in.header.frame_id[1:]

        try:            
            transform = self.listenerBuffer.lookup_transform(self.active_costmap.header.frame_id, pose_in.header.frame_id, now, rospy.Duration(4.0))
            pose_out = do_transform_pose(pose_in, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("[%s] transform from (%s) to (%s) failed: (%s).", rospy.get_name(), pose_in.header.frame_id, self.active_costmap.header.frame_id, e)        
        return pose_out
   
    def getCostmapValue(self, costmap,local_grid, px, py):
        isInside = False
        val = 0
        (ci, cj, isInside) = self.pose2cell(px , py, costmap)

        if isInside:
            val = local_grid[ci,cj] 

        return (px , py, ci, cj, val)

    def setCostmapValue(self,costmap,local_grid, px, py, val):
        isInside = False
        (ci, cj, isInside) = self.pose2cell(px,py, costmap)

        if isInside: 
            local_grid[ci,cj] = val
        return isInside

    def pose2cell(self,x,y, costmap):
        resolution = costmap.info.resolution
        height = costmap.info.height
        width = costmap.info.width

        i = j = 0
        isValid = True
        i = int((x / resolution)  + (width/2.0))   
        j = int((y / resolution)  + (height/2.0))  

        if (i > (width-1))  or (i<0) or (j > (height-1))  or (j<0):
            isValid = False
         
        return (i,j,isValid)

if __name__ == "__main__":
    rospy.init_node("task_cost_evaluator")#, log_level=rospy.DEBUG)
     # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        v = TaskCostEvaluator(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



