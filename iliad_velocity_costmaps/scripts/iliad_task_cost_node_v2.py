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
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseStamped, Quaternion

from bayes_people_tracker.msg import PeopleTracker
from orunav_msgs.msg import PoseSteering, ControllerReport, Task

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
        self.costmap = None
        self.costmap_update = None
        self.path_costmap = None
        self.path_costmap_update = None
        self.robotPoseSt = None
        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()
        
        rospy.loginfo("["+rospy.get_name()+"] " + "... all done.")

    def initROS(self):
        # publishers
        self.path_occ_grid_pub = rospy.Publisher(self.path_occ_grid_topic_name, OccupancyGrid, queue_size=1, latch=True)
        self.path_occ_grid_updates_pub = rospy.Publisher(self.path_occ_grid_topic_name + "_updates", OccupancyGridUpdate, queue_size=1, latch=True)

        self.curr_cost_topic_pub = rospy.Publisher(self.curr_cost_topic_name, Float64, queue_size=1, latch=True)
        # service clients
        
        # ... none here

        # subscribers and listeners
        self.listenerBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.listenerBuffer)
        rospy.Subscriber(self.task_topic_name, Task, self.task_callback, queue_size=1)                
        rospy.Subscriber(self.costmap_topic_name, OccupancyGrid, self.costmap_callback, queue_size=1)                
        rospy.Subscriber(self.costmap_topic_name+ "_updates", OccupancyGridUpdate, self.costmap_updates_callback, queue_size=1)                
        rospy.Subscriber(self.robot_pose_topic_name, PoseStamped, self.robot_pose_callback, queue_size=1)

        # service servers

        # ... none here
        # Timers
        rospy.Timer(rospy.Duration(self.update_publish_period), self.publish_updates, oneshot=False)
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

        # current robot position from tf tree
        self.robot_pose_topic_name = rospy.get_param(
            '~robot_pose_topic_name', '/robot' + str(self.robot_id) + '/robot_poseST')

        # current cost topic name 
        self.curr_cost_topic_name = rospy.get_param(
            '~curr_cost_topic_name', '/robot' + str(self.robot_id) + '/qsr/cost')

        self.update_publish_period = rospy.get_param('~update_publish_period', 0.05)

    # we subscribe to a pose publisher to get robot position
    def robot_pose_callback(self, msg):
        with self.lock:         
            self.robotPoseSt = msg

    def task_callback(self,msg):
        with self.lock: 
            self.active_task = msg
            self.active_task_time = rospy.Time.now()  
        rospy.logdebug("Node [" + rospy.get_name() + "] Detected a new active Task!")
        
    # we get a new constraints costmap. This should happen once.
    def costmap_callback(self,msg):
        with self.lock:         
            self.costmap = msg
            self.path_costmap = msg        
            self.path_costmap.header.stamp = rospy.Time.now()
            # costs are stored in updates, so this shouldn't be necesary...
            # self.path_costmap.data = np.full((self.path_costmap.info.width, self.path_costmap.info.height), 0).flatten(order='C')
        rospy.logdebug("Node ["+rospy.get_name()+"] " + "Received constraints costmap ")

        self.path_occ_grid_pub.publish(self.path_costmap)

    def costmap_updates_callback(self,msg):        
        with self.lock: 
            self.costmap_update = msg
            self.path_costmap_update = OccupancyGridUpdate()
            self.path_costmap_update.header.frame_id = msg.header.frame_id
            self.path_costmap_update.header.seq = msg.header.seq
            self.path_costmap_update.header.stamp = rospy.Time.now()
            self.path_costmap_update.width = msg.width
            self.path_costmap_update.height = msg.height
            self.path_costmap_update.x = msg.x
            self.path_costmap_update.y = msg.y
            self.path_costmap_update.data = np.zeros(self.path_costmap_update.width * self.path_costmap_update.height)

    def publish_updates(self,event):        
        with self.lock: 
            if ((not self.active_task == None) and (not self.costmap_update == None) and (not self.path_costmap_update == None) and (not self.robotPoseSt == None)):
                newPosePath = []
                newRelCellPath = []
                newCostPath = []
                pendingPath = self.active_task.path.path
                # get part of the path still to be covered
                i_path      = self.findClosestInPath(pendingPath,self.robotPoseSt)
                # Resample
                pendingPath = self.resamplePath(pendingPath, self.costmap.info.resolution)
                for i,posSteer in enumerate(pendingPath):
                    localPoseSt = self.poseSteer2costmapPose(posSteer)
                    (px, py, ci, cj, val)  = self.getCostmapUpdateValue(self.costmap, self.costmap_update, localPoseSt.pose.position.x , localPoseSt.pose.position.y)

                    self.setCostmapUpdateValue(self.path_costmap, self.path_costmap_update, px, py, val)
                    #self.setCostmapUpdateValue(self.path_costmap, self.path_costmap_update, px, py, 100)

                    # costs are only for points yet to be visited.
                    if i>i_path:
                        newPosePath.append((px, py))
                        newRelCellPath.append((ci,cj))
                        newCostPath.append(val)  

                #publish the update         
                self.path_occ_grid_updates_pub.publish(self.path_costmap_update)
                
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
                rospy.loginfo_throttle(2,"["+rospy.get_name()+"] " + "Remaining path cost per meter is: " + str(cost))
            else:
                rospy.loginfo_throttle(2,"Node [" + rospy.get_name() + "] Too soon for an update")
        
    def resamplePath(self, poseStPath, resolution):
        X = [poseStPath[0].pose.position.x]
        Y = [poseStPath[0].pose.position.y]

        for i in range(1,len(poseStPath)):
            X_p = X[-1]
            Y_p = Y[-1]
            X_i = poseStPath[i].pose.position.x
            Y_i = poseStPath[i].pose.position.y

            inc_x = X_i - X_p
            inc_y = Y_i - Y_p
            nx = np.ceil(abs(inc_x)/(resolution)).astype(int)
            ny = np.ceil(abs(inc_y)/(resolution)).astype(int)
            n = max(nx,ny)
            for j in range(1,n):
                X_n = X_p + inc_x * j / (n-1)
                Y_n = Y_p + inc_y * j / (n-1)
                X.append(X_n)                        
                Y.append(Y_n)
        
        newPoseStPath = []
        for x, y in zip(X, Y):
            p = PoseSteering()
            p.pose.position.x = x
            p.pose.position.y = y
            newPoseStPath.append(p)
        return  newPoseStPath 
    


    def findClosestInPath(self,poseSteerPath,robotposeSt):
        # Not the smartest search ...
        dist = 1e10
        ind = -1

        for i,posSteer in enumerate(poseSteerPath):
            path_poseSt_i= self.poseSteer2PoseSt(posSteer)

            dist_i = self.getDist(path_poseSt_i,robotposeSt)
            if (dist>dist_i):
                ind = i
                dist = dist_i
        return ind
    
    def getDist(self, poseStA, poseStB):
    
        if not (poseStA.header.frame_id == poseStB.header.frame_id):
            poseStA=self.transformPose(poseStB.header.frame_id, poseStA)

        dist = self.getDistPose(poseStA.pose, poseStB.pose)
        
        return dist
    
    def getDistPose(self, poseA, poseB):

        dist = np.sqrt( np.power(poseA.position.x-poseB.position.x, 2) +
                        np.power(poseA.position.y-poseB.position.y, 2) +
                        np.power(poseA.position.z-poseB.position.z, 2) )
        
        return dist
    
    def poseSteer2PoseSt(self, poseSteer):
        pose_in = PoseStamped()
        pose_in.pose = poseSteer.pose
        pose_in.header.frame_id = self.task_frame_id
        pose_in.header.stamp = self.active_task_time
        return pose_in

    def poseSteer2costmapPose(self, poseSteer):
        pose_in = self.poseSteer2PoseSt(poseSteer)
        pose_out = self.transformPose(self.costmap.header.frame_id, pose_in)
        return pose_out

    def transformPose(self, out_frame_id, pose_in):
        # in tf2, frames do not have the initial slash
        if (pose_in.header.frame_id[0] == '/'):
            pose_in.header.frame_id = pose_in.header.frame_id[1:]
        
        # in tf2, frames do not have the initial slash
        if (out_frame_id[0] == '/'):
            out_frame_id = out_frame_id[1:]

        try:            
            transform = self.listenerBuffer.lookup_transform(out_frame_id, pose_in.header.frame_id, rospy.Time.now(), rospy.Duration(4.0))
            pose_out = do_transform_pose(pose_in, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("[%s] transform from (%s) to (%s) failed: (%s).", rospy.get_name(), pose_in.header.frame_id, out_frame_id, e)
            pose_out = None        
        return pose_out
   
    def getCostmapUpdateValue(self, costmap,costmap_update, px, py):
        inside = False
        val = 0
        (ci, cj, inside) = self.pose2cell(px,py, costmap)
        # rospy.loginfo("["+rospy.get_name()+"] " + "Pose [" + '{0:.2f}'.format(px) + ", " + '{0:.2f}'.format(py) + "]")
        if inside: 
            # rospy.loginfo("["+rospy.get_name()+"] " + "Is cell [" + str(ci) + ", " + str(cj) + "]")            
            # cell pose, relative to update grid
            ci = ci - costmap_update.x 
            cj = cj - costmap_update.y 
            inside = self.isInside(ci,cj, costmap_update.width, costmap_update.height)
            if inside: 
                # rospy.loginfo("["+rospy.get_name()+"] " + "Is REL cell [" + str(ci) + ", " + str(cj) + "]")            
                k = self.linIndex(ci,cj, costmap_update.width)
                val = costmap_update.data[k]
        #         rospy.loginfo("["+rospy.get_name()+"] " + "Is lin Index [" + str(k) + "] == VAL [" + str(val) + "]")            
        #     else:
        #         rospy.logwarn("["+rospy.get_name()+"] " + "Is outside UPDATE")
        # else:
        #     rospy.logerr("["+rospy.get_name()+"] " + "Is outside costmap")

        return (px , py, ci, cj, val)

    def setCostmapUpdateValue(self,costmap,costmap_update, px, py, val):
        inside = False
        (ci, cj, inside) = self.pose2cell(px,py, costmap)
        if inside: 
            # cell pose, relative to update grid
            ci = ci - costmap_update.x 
            cj = cj - costmap_update.y 
            inside = self.isInside(ci,cj, costmap_update.width, costmap_update.height)
            if inside: 
                k = self.linIndex(ci,cj, costmap_update.width)
                costmap_update.data[k] = val

        return inside

    def linIndex(self,i,j,width):
        k = i + j * width
        return k

    def pose2cell(self,x,y, costmap):
        ix, iy = self.getRel(x,y,costmap)
        i = int((ix / costmap.info.resolution))   
        j = int((iy / costmap.info.resolution))  

        isValid = self.isInside(i,j, costmap.info.width, costmap.info.height)
         
        return (i,j,isValid)

    def isInside(self, i,j, w, h):
        isValid = True
        if (i > (w-1))  or (i<0) or (j > (h-1))  or (j<0):
            isValid = False
        return isValid

    def getRel(self,x,y,costmap):
        ox = costmap.info.origin.position.x 
        oy = costmap.info.origin.position.y 

        #MFC: layered costmap kind of assumes same position and orientation in all static maps... so DON'T use rotations!
        # oa = self.get_rotation(costmap.info.origin.orientation)
        # MFC: layered costmap kind of assumes same position and orientation in all static maps... so DON'T rotate!
        dx = (x - ox)
        dy = (y - oy)
        # nx =  dx * np.cos(oa) + dy * np.sin(oa) 
        # ny = -dx * np.sin(oa) + dy * np.cos(oa) 
        return (dx,dy)
        
if __name__ == "__main__":
    rospy.init_node("task_cost_evaluator")#, log_level=rospy.DEBUG)
     # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        v = TaskCostEvaluator(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



