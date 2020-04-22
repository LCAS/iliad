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
from geometry_msgs.msg import Pose, PointStamped, PoseStamped, Quaternion

from bayes_people_tracker.msg import PeopleTracker

import numpy as np
from threading import Lock


class ConstraintsCostmapV1(object):
    """


    """

    def __init__(self, costmap_topic_pub, ref_costmap):

        self.frame_id = ref_costmap.header.frame_id
        self.resolution = ref_costmap.info.resolution
        # self.height = int(20.0 / ref_costmap.info.resolution) #y
        # self.width  = int(26.0 / ref_costmap.info.resolution) #x
        self.seq = 0
        self.height = ref_costmap.info.height
        self.width  = ref_costmap.info.width 


        self.local_data = np.full( (self.width, self.height), 0)

        self.lock = Lock()

        self.local_human_pose = None
        self.local_robot_pose = None

        self.map_pub = costmap_topic_pub

        rospy.Timer(rospy.Duration(0.2), self.update_map, oneshot=False)

        # Useful for debugging
        # rospy.Subscriber("/clicked_point", PointStamped, self.point_callback, queue_size=1)

        # need to figure out a better way to do this
        self.listenerBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.listenerBuffer)

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

        if (i > (self.width-1))  or (i<0) or (j > (self.height-1))  or (j<0):
            isValid = False
         
        return (i,j,isValid)
    
    def getRel(self,x,y):
        dx = (x - self.ox)
        dy = (y - self.oy)
        nx = dx * np.cos(self.oa) + dy * np.sin(self.oa) 
        ny = -dx * np.sin(self.oa) + dy * np.cos(self.oa) 
        return (nx,ny)

    def cell2pose(self,i,j):
        x = y = np.nan
        isValid = True

        if (i > (self.width-1))  or (i<0) or (j > (self.height-1))  or (j<0):
            isValid = False

        x = ( i - (self.width/2.0)  ) * self.resolution
        y = ( j - (self.height/2.0) ) * self.resolution
         
        return (x,y,isValid)

    def setValue(self, px, py, val):
        # px and py are assumed to be in self.frame_id
        isInside = False
        (ci, cj, isInside) = self.pose2cell(px,py)

        if isInside:
            #self.local_data = self.current_occ_grid.data.reshape(self.height,self.width).T
           
            self.local_data[ci,cj] = val

            #self.current_occ_grid.data =  self.local_data.T.flatten(order='C')
            # rospy.loginfo("Node [" + rospy.get_name() + "] " + 
            #                 "Point ( " +
            #                 str(px) + ", " + str(py) + ") [" +                            
            #                 self.local_robot_pose.header.frame_id + "]" + 
            #                 " is in Cell (" + str(ci) + ", " + str(cj) + ") "                                                         
            #                 ) 
        return isInside

    def get_rotation (self,orientation_q):
        (roll, pitch, yaw) = euler_from_quaternion ([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        return yaw

    def update_map(self,event):
        with self.lock: 
            if (not self.local_human_pose == None) and (not self.local_robot_pose == None):
                now = rospy.Time.now()

                # create occ grid
                self.current_occ_grid = OccupancyGrid()
                self.current_occ_grid.info.resolution = self.resolution
                self.current_occ_grid.info.height = self.height
                self.current_occ_grid.info.width = self.width

                self.setOrigin()

                for ci in range(0,20):
                    for cj in range(0,20):
                        self.local_data[int(self.width/2.0)+ci-10,int(self.height/2.0)+cj-10] = 100

                # for ci in range(0,20):
                #     for cj in range(0,20):
                #         self.local_data[ci,cj] = 100

                ## get human distance to robot
                # xh = self.local_human_pose.pose.position.x 
                # yh = self.local_human_pose.pose.position.y
                

                # dh = np.sqrt(np.power(self.xx-xh,2)+np.power(self.yy-yh,2))
                # cd = np.exp(-np.power(0.75*(dh),2))
                
                # # equalize costs
                # cd = np.interp(cd, (cd.min(), cd.max()), (0, 1))
                
                # # combine costs
                # c = cd
                
                # # remap 0-100
                # c = np.interp(c, (c.min(), c.max()), (0, 100))


                # published data needs to be flattened ...
                self.current_occ_grid.data =  self.local_data.T.flatten(order='C')#   .astype(np.uint8) # is datetype necessary??
           
                self.map_pub.publish(self.current_occ_grid)
                dur = rospy.Time.now() - now
                # rospy.logdebug_throttle(2, "Node [" + rospy.get_name() + "] " +
                #                 "Map published in (" + str(dur.to_sec()) + ") secs"
                #                 )

    def setOrigin(self):
        # we assume robot pose is in self.frame_id...
        self.current_occ_grid.header.frame_id = self.frame_id       
        self.current_occ_grid.header.stamp = rospy.Time.now()
        self.current_occ_grid.header.seq = self.seq
        self.seq = self.seq + 1
        self.current_occ_grid.info.origin =  self.local_robot_pose.pose
        ang = self.get_rotation(self.local_robot_pose.pose.orientation)

        dy = (self.width/2.0) * self.resolution 
        dx = (self.height/2.0)  * self.resolution
        al = np.arctan2(dx,dy)
        b = np.pi/2.0 - ang - al
        r = np.sqrt(np.power( dx ,2.0) + np.power( dy ,2.0))
        iy = r * np.cos(b)
        ix = r * np.sin(b)
        self.current_occ_grid.info.origin.position.x -= ix
        self.current_occ_grid.info.origin.position.y -= iy

        self.ox = self.current_occ_grid.info.origin.position.x
        self.oy = self.current_occ_grid.info.origin.position.y
        self.oa = ang
        # mapOrig = PoseStamped()
        # mapOrig.pose = self.current_occ_grid.info.origin
        # mapOrig.header = self.current_occ_grid.header
        # self.printPoseSt(mapOrig, "map orig:")
        # rospy.loginfo("Node [" + rospy.get_name() + "] " +
        #                 "dx = " + '{0:.2f}'.format(dx) )
        # rospy.loginfo("Node [" + rospy.get_name() + "] " +
        #                 "dy = " + '{0:.2f}'.format(dy) )
        # rospy.loginfo("Node [" + rospy.get_name() + "] " +
        #                 "ang = " + '{0:.2f}'.format(ang) )        


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
            rospy.logerr("[%s] transform from (%s) to (%s) failed: (%s).", rospy.get_name(), pose_in.header.frame_id, new_frame, e)        
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
            self.icc = ConstraintsCostmapV1(self.costmap_topic_pub, self.map)
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
            rospy.logerr("[%s] transform from (%s) to (%s) failed: (%s).", rospy.get_name(), pose_in.header.frame_id, new_frame, e)        
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



