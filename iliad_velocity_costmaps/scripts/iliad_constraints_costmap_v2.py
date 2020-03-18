#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

This version creates a costmap with same size, resolution and frame id than static navigation map

"""

import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from message_filters import Subscriber
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Pose, PointStamped, PoseStamped, Quaternion

from bayes_people_tracker.msg import PeopleTracker

import numpy as np
from threading import Lock
from scipy.sparse import *


class ConstraintsCostmapV2(object):
    """


    """

    def __init__(self, costmap_topic_pub, costmap_updates_topic_pub, ref_costmap):

        self.current_occ_grid = ref_costmap
    
        # these are for easiness...
        self.frame_id = ref_costmap.header.frame_id
        self.resolution = ref_costmap.info.resolution
        self.seq = 0

        self.height = ref_costmap.info.height
        self.width  = ref_costmap.info.width 

        self.ox = self.current_occ_grid.info.origin.position.x 
        self.oy = self.current_occ_grid.info.origin.position.y 
        self.oa = self.get_rotation(self.current_occ_grid.info.origin.orientation)

        # odd stuff happens if you use different static map sizes...
        # self.height = int(20.0 / self.resolution) #y
        # self.width  = int(26.0 / self.resolution) #x
        # self.current_occ_grid.info.height = self.height
        # self.current_occ_grid.info.width = self.width

        # Translate to be centered under costmap_frame_id
        # layered costmap kind of assumes same position and orientation in all static maps... so DON'T!
        # self.current_occ_grid.info.origin = Pose()        
        # self.current_occ_grid.info.origin.position.x = -(self.width/2.0)   * self.resolution 
        # self.current_occ_grid.info.origin.position.y = -(self.height/2.0)  * self.resolution


        # clear data
        self.current_occ_grid.data = np.zeros(self.width * self.height)
        #self.local_data = np.full( (self.width, self.height), 0)
        
        #self.local_data = lil_matrix( (self.width, self.height), dtype=np.uint8 )
        #self.local_data = csr_matrix( (self.width, self.height), dtype=np.uint8 )

        self.lock = Lock()

        self.local_human_pose = None
        self.local_robot_pose = None
        self.last_update = None
        self.map_pub = costmap_topic_pub
        self.update_map_pub = costmap_updates_topic_pub

        self.publish_map()
        
        rospy.Timer(rospy.Duration(0.05), self.update_map, oneshot=False)

        # Useful for debugging
        # rospy.Subscriber("/clicked_point", PointStamped, self.point_callback, queue_size=1)

        # need to figure out a better way to do this
        self.listenerBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.listenerBuffer)


    def publish_map(self):
        with self.lock: 
                now = rospy.Time.now()
                # ..........................................
                self.current_occ_grid.header.stamp = rospy.Time.now()
                self.current_occ_grid.header.seq = 0
                self.map_pub.publish(self.current_occ_grid)
                # ..........................................
                dur = rospy.Time.now() - now
                rospy.logdebug("Node [" + rospy.get_name() + "] " +
                                "Init Map published in (" + str(dur.to_sec()) + ") secs"
                                )  

    # Useful for debugging
    # def point_callback(self,pointSt):
    #     poseIn = PoseStamped()
    #     poseIn.pose.position =pointSt.point
    #     poseIn.header =pointSt.header
    #     poseLocal = self.cast_pose(poseIn, self.frame_id)
    #     ans = self.setValue(poseLocal.pose.position.x,poseLocal.pose.position.y,100)
    #     if ans:
    #         self.printPoseSt(poseLocal, "Changed Point:")
    #     else:
    #         self.printPoseSt(poseLocal, "OUT OF THE MAP!!!:")            
    
    def update_human(self, humanPoseSt):
        if not (humanPoseSt == None):
            self.local_human_pose = humanPoseSt            
            #self.printPoseSt(humanPoseSt, "human pose received")

    def update_robot(self, robotPoseSt):
        if not (robotPoseSt == None):
            self.local_robot_pose = robotPoseSt            
            #self.printPoseSt(robotPoseSt, "robot pose received")


    def pose2cell(self,x,y):
        i = j = 0
        isValid = True
        ix, iy = self.getRel(x,y)
        i = int((ix / self.resolution))   
        j = int((iy / self.resolution))  

        isValid = self.isInside(i,j)
         
        return (i,j,isValid)
    
    def getRel(self,x,y):
        # layered costmap kind of assumes same position and orientation in all static maps... so DON'T rotate!
        dx = (x - self.ox)
        dy = (y - self.oy)
        # nx = dx * np.cos(self.oa) + dy * np.sin(self.oa) 
        # ny = -dx * np.sin(self.oa) + dy * np.cos(self.oa) 
        return (dx,dy)

    def cell2pose(self,i,j):
        x = y = np.nan
        
        isValid = self.isInside(i,j)

        x = ( i - (self.width/2.0)  ) * self.resolution
        y = ( j - (self.height/2.0) ) * self.resolution
         
        return (x,y,isValid)

    def isInside(self, i,j):
        return self.isInside0(i,j,self.width,self.height)

    def isInside0(self, i,j,w,h):
        isValid = True
        if (i > (w-1))  or (i<0) or (j > (h-1))  or (j<0):
            isValid = False
        return isValid

    def setValue(self, px, py, val):
        # px and py are assumed to be in self.frame_id
        isInside = False
        (ci, cj, isInside) = self.pose2cell(px,py)

        if isInside:
            self.local_data[ci,cj] = val
        return isInside
    
    def linIndex(self,i,j):        
        return self.linIndex0(i,j,self.width)

    def linIndex0(self,i,j,width):
        k = i + j * width
        return k

    def cellIndex(self,k):        
        return self.cellIndex0(k,self.width)
    
    def cellIndex0(self,k,width):
        (i,j)  =  (  (k%width), int(k/width)  )
        return (i,j)

    def get_rotation (self,orientation_q):
        (roll, pitch, yaw) = euler_from_quaternion ([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        return yaw

    def update_map(self,event):
        with self.lock: 
            if (not self.local_human_pose == None) and (not self.local_robot_pose == None):

                # if (not self.last_update==None):
                #     # clearing
                #     # self.last_update.x -= 100
                #     # self.last_update.y -= 100
                #     # self.last_update.width  += 100 
                #     # self.last_update.height += 100                   
                #     self.last_update.data = np.zeros(self.last_update.width * self.last_update.height )
                #     self.update_map_pub.publish(self.last_update) 


                now = rospy.Time.now()
                # ............................

                update = OccupancyGridUpdate()
                update.header = self.current_occ_grid.header
                update.header.stamp = rospy.Time.now()
                update.header.seq = self.seq
                self.seq = self.seq + 1

                # random update grid start position
                (ih0,jh0,valid) = self.pose2cell(15.3,-3.03)
                update.width  = 375
                update.height = 306 

                # get gridmap human pose
                xh = self.local_human_pose.pose.position.x 
                yh = self.local_human_pose.pose.position.y
                (ih,jh,valid) = self.pose2cell(xh,yh)

                # update grid starts here                
                update.x = ih0 - update.width/2
                update.y = jh0 - update.height/2


                update.data = np.zeros(update.width * update.height)

                # this is human cell, relative to update grid
                ih = ih - update.x 
                jh = jh - update.y 

                # mark margins
                # update.data[0] = 100
                # update.data[update.width*update.height-1] = 100

                # this creates a square around human
                for i in range(0,update.width):
                    for j in range(0,update.height):                        
                        k = self.linIndex0(i,j, update.width)
                        if (abs(ih-i)<int(update.width/20)) and (abs(jh-j)<int(update.height/20)):
                            update.data[k] = 100

                
                            
                self.last_update = update
                self.update_map_pub.publish(update) 

                # ..........................................
                dur = rospy.Time.now() - now
                rospy.logdebug("Node [" + rospy.get_name() + "] " +
                                "Map update published in (" + str(dur.to_sec()) + ") secs"
                                )
                #self.printPoseSt(self.local_human_pose,"Human at")                

    def update_mapO(self,event):
        with self.lock: 
            if (not self.local_human_pose == None) and (not self.local_robot_pose == None):
                now = rospy.Time.now()

                # update occ grid
                self.current_occ_grid.header.stamp = rospy.Time.now()
                self.current_occ_grid.header.seq = self.seq
                self.seq = self.seq + 1

                # draw in the grid center
                # maxI=20
                # maxJ=int(now.to_sec()%80)
                # for ci in range(0,maxI):
                #     for cj in range(0,maxJ):
                #         self.local_data[int(self.width/2.0)+ci-maxI/2,int(self.height/2.0)+cj-maxJ/2] = 100


                # draw in a corner
                # for ci in range(0,20):
                #     for cj in range(0,20):
                #         self.local_data[ci,cj] = 100

                
                xh = self.local_human_pose.pose.position.x 
                yh = self.local_human_pose.pose.position.y
                maxI=20
                maxJ=int(now.to_sec()%80)
                (ih,jh,valid) = self.pose2cell(xh,yh)
                if valid:
                    for ci in range(0,maxI):
                        for cj in range(0,maxJ):
                            i = ih + ci - maxI/2
                            j = jh + cj - maxJ/2
                            if self.isInside(i,j):
                                #self.local_data[i,j] = 100   
                                self.current_occ_grid.data[self.linIndex(i,j)] = 100
                else:
                    self.printPoseSt(self.local_human_pose,"is Out!")
                    rospy.loginfo(" == CELL [" + str(i) + ", " + str(j) + "] " 
                    )
                # dh = np.sqrt(np.power(self.xx-xh,2)+np.power(self.yy-yh,2))
                # cd = np.exp(-np.power(0.75*(dh),2))
                
                # # equalize costs
                # cd = np.interp(cd, (cd.min(), cd.max()), (0, 1))
                
                # # combine costs
                # c = cd
                
                # # remap 0-100
                # c = np.interp(c, (c.min(), c.max()), (0, 100))


                # published data needs to be flattened ...
                #k = np.array(self.local_data.todense())
                #self.current_occ_grid.data =  k.T.flatten(order='C')#   .astype(np.uint8) # is datetype necessary??
           
                self.map_pub.publish(self.current_occ_grid)
                dur = rospy.Time.now() - now
                rospy.logdebug_throttle(3, "Node [" + rospy.get_name() + "] " +
                                "Map published in (" + str(dur.to_sec()) + ") secs"
                                )

    def get_quaternion(self,yaw):
        q_angle = quaternion_from_euler(0, 0, yaw, axes='sxyz') # is axes ok ??
        q = Quaternion(*q_angle)
        return q

    def printPoseSt(self, poseSt,text):
        rospy.loginfo("Node [" + rospy.get_name() + "] " +
                text + ": Pose ( " +
                '{0:.2f}'.format(poseSt.pose.position.x) + ", " + '{0:.2f}'.format(poseSt.pose.position.y) + ", " +
                '{0:.2f}'.format(self.get_rotation(poseSt.pose.orientation) * 180.0 /np.pi) + " deg) " +
                " in frame (" + poseSt.header.frame_id+ ")"
                )      

    def cast_pose(self, pose_in, new_frame):
        pose_out = None
        now = rospy.Time.now()

        # in tf2, frames do not have the initial slash
        if (pose_in.header.frame_id[0] == '/'):
            pose_in.header.frame_id = pose_in.header.frame_id[1:]

        # in tf2, frames do not have the initial slash
        if (new_frame[0] == '/'):
            new_frame = new_frame[1:]

        try:            
            transform = self.listenerBuffer.lookup_transform(new_frame, pose_in.header.frame_id, now, rospy.Duration(4.0))
            pose_out = do_transform_pose(pose_in, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("[%s] transform from (%s) to (%s) failed: (%s).", rospy.get_name(), pose_in.header.frame_id, new_frame, e)        
        return pose_out


class IliadConstraintsCostmapServerV1(object):

    def __init__(self, name):
        rospy.loginfo("["+rospy.get_name()+"] " + "Starting ... ")

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # other params
        self.gotMap = False
        
        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("["+rospy.get_name()+"] " + "... all done.")

    def initROS(self):
        # publishers
        self.costmap_topic_pub = rospy.Publisher(self.costmap_topic_name, OccupancyGrid, queue_size=10, latch=True)
        self.costmap_updates_topic_pub = rospy.Publisher(self.costmap_topic_name + "_updates", OccupancyGridUpdate, queue_size=10, latch=True)
        # service clients
        
        # ... none here

        # subscribers and listeners
        rospy.Subscriber(self.human_tracking_topic_name, PeopleTracker, self.human_tracking_callback, queue_size=1)
        
        rospy.Subscriber(self.robot_pose_topic_name, PoseStamped, self.robot_pose_callback, queue_size=1)

        rospy.Subscriber(self.map_topic_name, OccupancyGrid, self.map_callback, queue_size=1)

        self.listenerBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.listenerBuffer)
        
        # service servers

        # ... none here

    def map_callback(self, map):
        self.map = map
        if not self.gotMap:
            self.icc = ConstraintsCostmapV2(self.costmap_topic_pub, self.costmap_updates_topic_pub, self.map)
            rospy.loginfo("["+rospy.get_name()+"] " + "Map received. Creating costmap.")
        self.gotMap = True
    # .............................................................................................................

    def loadROSParams(self):

        self.robot_id = rospy.get_param('~robot_id', 4)

        # Output Costmap topic
        self.costmap_topic_name = rospy.get_param(
            '~costmap_topic_name', '/robot'+str(self.robot_id)+'/qsr/constraints_costmap')

        # Input map topic
        self.map_topic_name = rospy.get_param(
            '~map_topic_name', '/map_laser2d')

        # human tracking trackedpersons (SPENCER) topic
        self.human_tracking_topic_name = rospy.get_param(
            '~human_tracking_topic_name', '/robot'+str(self.robot_id)+'/qsr/people_tracker/positions')
         

        # current robot position from tf tree
        self.robot_pose_topic_name = rospy.get_param(
            '~robot_pose_topic_name', '/robot' + str(self.robot_id) + '/robot_poseST')

    def human_tracking_callback(self, ppl):
        try:
           min_index = ppl.distances.index(ppl.min_distance)
        except ValueError as ex:
           if len(ppl.distances)>0:
              min_index = ppl.distances.index(min(ppl.distances))
           else:
              #rospy.logerr("Node [" + rospy.get_name() + "] " + "people tracker distance vector empty! ")
              return
        min_pose = ppl.poses[min_index]
        min_ang = ppl.angles[min_index]
        # not used ... by now
        min_vel = ppl.velocities[min_index]

        humanPoseSt = PoseStamped()
        humanPoseSt.header = ppl.header
        humanPoseSt.pose = min_pose
        
        try:
            # we could have potentially several maps here, each one tracking a different human
            self.icc.update_human(self.get_local_pose(humanPoseSt, self.map.header.frame_id))
        except AttributeError as ae:
            # first msg may arrive even before the map....
            pass

    def robot_pose_callback(self, msg):
        self.robotPoseSt = msg

        try:
            self.icc.update_robot(self.get_local_pose(self.robotPoseSt, self.map.header.frame_id))
        except AttributeError as ae:
            # first msg may arrive even before creating the map....
            pass

    def get_local_pose(self, pose_in, new_frame):
        pose_out = None
        now = rospy.Time.now()

        # in tf2, frames do not have the initial slash
        if (pose_in.header.frame_id[0] == '/'):
            pose_in.header.frame_id = pose_in.header.frame_id[1:]
        
        # in tf2, frames do not have the initial slash
        if (new_frame[0] == '/'):
            new_frame = new_frame[1:]            

        try:            
            transform = self.listenerBuffer.lookup_transform(new_frame, pose_in.header.frame_id, now, rospy.Duration(4.0))
            pose_out = do_transform_pose(pose_in, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("[%s] transform from (%s) to (%s) failed: (%s).", rospy.get_name(), pose_in.header.frame_id, new_frame, e)        
        return pose_out

    def get_quaternion(self,yaw):
        q_angle = quaternion_from_euler(0, 0, yaw, axes='sxyz') # is axes ok ??
        q = Quaternion(*q_angle)
        return q

    def get_rotation (self,orientation_q):
        (roll, pitch, yaw) = euler_from_quaternion ([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        return yaw        

if __name__ == "__main__":
    rospy.init_node("constraints_costmap_server_vv1" , log_level=rospy.DEBUG)
     # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        v = IliadConstraintsCostmapServerV1(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



