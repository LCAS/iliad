#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Creates a costmap at human pose pointing towards robot.
Costmap is a triangle pointing one vertex towards robot.


"""

import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from message_filters import Subscriber
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, Quaternion

from bayes_people_tracker.msg import PeopleTracker
from orunav_msgs.msg import ControllerReport

import numpy as np
from threading import Lock


class ConstraintsCostmapV0(object):
    """
    Costmap logic lies here
    """

    def __init__(self, costmap_frame_id, costmap_topic_pub, width=512, height=512,resolution=0.05):

        o = OccupancyGrid()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = costmap_frame_id
        o.info.resolution = resolution
        o.info.height = height #y
        o.info.width = width #x
        o.info.origin = Pose()

        # Translate to be centered under costmap_frame_id
        o.info.origin.position.x -= (o.info.width/2.0) * o.info.resolution 
        o.info.origin.position.y -= (o.info.height/2.0)  * o.info.resolution
        o.data = np.full( (o.info.width, o.info.height), 0).T.flatten(order='C')
     
        self.current_occ_grid = o
        self.lock = Lock()

        self.local_human_pose = None
        self.local_robot_pose = None

        self.map_pub = costmap_topic_pub

        rospy.Timer(rospy.Duration(0.2), self.update_map, oneshot=False)


        # plot meshes
        x = np.linspace(-(o.info.width/2.0)* o.info.resolution , (o.info.width/2.0)* o.info.resolution , o.info.width)        
        y = np.linspace(-(o.info.height/2.0)* o.info.resolution , (o.info.height/2.0)* o.info.resolution , o.info.height)        
        self.xx, self.yy = np.meshgrid(x, y)
        self.r = np.sqrt(np.power(self.xx,2)+np.power(self.yy,2))
        self.th = np.arctan2(self.yy,self.xx)

        # need to figure out a better way to do this
        self.listenerBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.listenerBuffer)
    
    def update_human(self, humanPoseSt):
        if not (humanPoseSt == None):
            self.local_human_pose = humanPoseSt
        rospy.logdebug_throttle(2, "Node [" + rospy.get_name() + "] " +
                                "Human pose received"
                                )

    def update_robot(self, robPoseSt):
        if not (robPoseSt == None):
            self.local_robot_pose = robPoseSt            
        rospy.logdebug_throttle(2, "Node [" + rospy.get_name() + "] " +
                                "robot pose received"
                                )

    def pose2cell(self,x,y):
        resolution = self.current_occ_grid.info.resolution
        height = self.current_occ_grid.info.height
        width = self.current_occ_grid.info.width
        # width=40, height=60,resolution=0.5
        # origin at 
        # [0,             0] is frame_id: "robot4/base_link"   x: -9.5   y: -14.77
        # [width-1,       0] is frame_id: "robot4/base_link"   x:  9.5   y: -14.77
        # [width-1,height-1] is frame_id: "robot4/base_link"   x:  9.9   y:  14.77 
        # [0,      height-1] is frame_id: "robot4/base_link"   x: -9.5   y:  14.77 

        i = j = 0
        isValid = True
        i = int((x / resolution)  + (width/2.0))   
        j = int((y / resolution)  + (height/2.0))  

        if (i > (width-1))  or (i<0) or (j > (height-1))  or (j<0):
            isValid = False
         
        return (i,j,isValid)

    def cell2pose(self,i,j):
        resolution = self.current_occ_grid.info.resolution
        height = self.current_occ_grid.info.height
        width = self.current_occ_grid.info.width

        x = y = np.nan
        isValid = True

        if (i > (width-1))  or (i<0) or (j > (height-1))  or (j<0):
            isValid = False

        x = ( i - (width/2.0)  ) * resolution
        y = ( j - (height/2.0) ) * resolution

         
        return (x,y,isValid)

    def setValue(self, px, py, val):
        # this method seems to me like overkilling: two reshapes and transposes...
        isInside = False
        (ci, cj, isInside) = self.pose2cell(px,py)

        if isInside:
            height = self.current_occ_grid.info.height
            width = self.current_occ_grid.info.width
            local_grid = self.current_occ_grid.data.reshape(height,width).T
           
            local_grid[ci,cj] = val

            self.current_occ_grid.data =  local_grid.T.flatten(order='C')
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
            if ((not self.local_human_pose == None) and (not self.local_robot_pose == None)):
                height = self.current_occ_grid.info.height
                width = self.current_occ_grid.info.width
                ## get human distance to map origin
                xh = self.local_human_pose.pose.position.x 
                yh = self.local_human_pose.pose.position.y
                #rh = np.sqrt(np.power(xh,2)+np.power(yh,2))
                thh= np.arctan2(yh,xh)

                dh = np.sqrt(np.power(self.xx-xh,2)+np.power(self.yy-yh,2))
                
                ca = np.exp(-np.power(4*(self.th-thh),2))
                cd = np.exp(-np.power(0.75*(dh),2))
                # equalize costs
                ca = np.interp(ca, (ca.min(), ca.max()), (0, 10))
                cd = np.interp(cd, (cd.min(), cd.max()), (0, 1))
                # combine costs
                c = ca * cd
                # remap 0-100
                c = np.interp(c, (c.min(), c.max()), (0, 100))
                self.current_occ_grid.data =  c.flatten(order='C')

                #self.setValue(self.local_human_pose.pose.position.x , self.local_human_pose.pose.position.y, 100)
                #self.setValue(self.local_robot_pose.pose.position.x , self.local_robot_pose.pose.position.y, 100)

                # self.current_occ_grid.info.origin.position.x = (self.current_occ_grid.info.width/2.0)   * self.current_occ_grid.info.resolution 
                # self.current_occ_grid.info.origin.position.y = (self.current_occ_grid.info.height/2.0)  * self.current_occ_grid.info.resolution

                #self.awfulCaster()

                self.map_pub.publish(self.current_occ_grid)
                rospy.logdebug_throttle(2, "Node [" + rospy.get_name() + "] " +
                                "Map published"
                                )

            # if (not self.local_robot_pose == None):
            #     rospy.loginfo("Node [" + rospy.get_name() + "] " +
            #                     "update_robot status: Pose ( " +
            #                     str(self.local_robot_pose.pose.position.x) + ", " + str(self.local_robot_pose.pose.position.y) + ", " +
            #                     str(self.get_rotation(self.local_robot_pose.pose.orientation) * 180.0 /np.pi) + " deg) "  +
            #                     " in frame (" + self.local_robot_pose.header.frame_id+ ")"
            #                     ) 
            # else:
            #     rospy.logerr("Node [" + rospy.get_name() + "] " +
            #                     "update_robot status: Pose ( None )") 
            # if (not self.local_human_pose == None):                                
            #     rospy.loginfo("Node [" + rospy.get_name() + "] " +
            #                     "update_human status: Pose ( " +
            #                     str(self.local_human_pose.pose.position.x) + ", " + str(self.local_human_pose.pose.position.y) + ", " +
            #                     str(self.get_rotation(self.local_human_pose.pose.orientation) * 180.0 /np.pi) + " deg) " +
            #                     " in frame (" + self.local_human_pose.header.frame_id+ ")"
            #                     )                                
            # else:
            #     rospy.logerr("Node [" + rospy.get_name() + "] " +
            #                     "update_human status: Pose ( None )") 

    def awfulCaster(self):
        # ................................
        oldOrigin = PoseStamped()
        oldOrigin = self.local_robot_pose
        oldOrigin.pose.position.x -= (self.current_occ_grid.info.width/2.0) * self.current_occ_grid.info.resolution 
        oldOrigin.pose.position.y -= (self.current_occ_grid.info.height/2.0)  * self.current_occ_grid.info.resolution


        newOrigin = self.cast_pose(oldOrigin, "map_laser2d")
        self.current_occ_grid.info.origin = newOrigin.pose
        self.current_occ_grid.header.frame_id = "map_laser2d"
        self.current_occ_grid.header.stamp = rospy.Time.now()


    def awfulCaster0(self):
        # ................................
        oldOrigin = PoseStamped()
        #oldOrigin.pose = self.current_occ_grid.info.origin
        oldOrigin.header.stamp = rospy.Time.now()
        oldOrigin.header.frame_id = "map_laser2d"

        newOrigin = self.cast_pose(oldOrigin, self.current_occ_grid.header.frame_id )
        self.current_occ_grid.info.origin = newOrigin.pose
        self.current_occ_grid.header.frame_id = "map_laser2d"
        self.current_occ_grid.header.stamp = rospy.Time.now()


    def cast_pose(self, pose_in, new_frame):
        pose_out = None
        now = rospy.Time.now()

        # in tf2, frames do not have the initial slash
        if (pose_in.header.frame_id[0] == '/'):
            pose_in.header.frame_id = pose_in.header.frame_id[1:]

        try:            
            transform = self.listenerBuffer.lookup_transform(new_frame, pose_in.header.frame_id, now, rospy.Duration(4.0))
            pose_out = do_transform_pose(pose_in, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("[%s] transform from (%s) to (%s) failed: (%s).", rospy.get_name(), pose_in.header.frame_id, new_frame, e)        
        return pose_out


class IliadConstraintsCostmapServer(object):
    def __init__(self, name):
        rospy.loginfo("["+rospy.get_name()+"] " + "Starting ... ")

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # other params
        self.robotPoseSt = PoseStamped()     
        self.robotPoseSt.header.stamp = rospy.Time.now()
        self.robotPoseSt.header.frame_id = self.reports_frame_id

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        self.icc = ConstraintsCostmapV0(self.costmap_frame_id, self.costmap_topic_pub)
        
        rospy.loginfo("["+rospy.get_name()+"] " + "... all done.")

    def initROS(self):
        # publishers
        self.costmap_topic_pub = rospy.Publisher(self.costmap_topic_name, OccupancyGrid, queue_size=10, latch=True)

        # service clients
        
        # ... none here

        # subscribers and listeners
        rospy.Subscriber(self.human_tracking_topic_name, PeopleTracker, self.human_tracking_callback, queue_size=1)
        rospy.Subscriber(self.reports_topic_name, ControllerReport, self.reports_callback, queue_size=1)
                
        self.listenerBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.listenerBuffer)
        
        # service servers

        # ... none here

    # .............................................................................................................

    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)

        # Costmap topic
        self.costmap_topic_name = rospy.get_param(
            '~costmap_topic_name', '/robot'+str(self.robot_id)+'/qsr/constraints_costmap')

        # Costmap frame id
        self.costmap_frame_id = rospy.get_param(
            '~costmap_frame_id', '/robot'+str(self.robot_id)+'/base_link')
        # in tf2, frames do not have the initial slash
        if (self.costmap_frame_id[0] == '/'):
            self.costmap_frame_id = self.costmap_frame_id[1:]

        # reports frame id
        self.reports_frame_id = rospy.get_param(
            '~reports_frame_id', '/world')

        # MPC reports with current robot state
        self.reports_topic_name = rospy.get_param(
            '~reports_topic_name', '/robot' + str(self.robot_id) + '/control/controller/reports')

        # human tracking trackedpersons (SPENCER) topic
        self.human_tracking_topic_name = rospy.get_param(
            '~human_tracking_topic_name', '/robot'+str(self.robot_id)+'/qsr/people_tracker/positions')

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
            self.icc.update_human(self.get_local_pose(humanPoseSt, self.costmap_frame_id))
        except AttributeError as ae:
            # first msg may arrive even before creating the map....
            pass

    # we use robot reports to know robot position
    def reports_callback(self, msg):
        self.robotPoseSt.header.seq = self.robotPoseSt.header.seq + 1
        self.robotPoseSt.header.stamp = rospy.Time.now()

        self.robotPoseSt.pose.position.x = msg.state.position_x
        self.robotPoseSt.pose.position.y = msg.state.position_y
        self.robotPoseSt.pose.orientation = self.get_quaternion(msg.state.orientation_angle)

        try:
            self.icc.update_robot(self.get_local_pose(self.robotPoseSt, self.costmap_frame_id))
        except AttributeError as ae:
            # first msg may arrive even before creating the map....
            pass

    def get_local_pose(self, pose_in, new_frame):
        pose_out = None
        now = rospy.Time.now()

        # in tf2, frames do not have the initial slash
        if (pose_in.header.frame_id[0] == '/'):
            pose_in.header.frame_id = pose_in.header.frame_id[1:]

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
    rospy.init_node("constraints_costmap_server_v0")#, log_level=rospy.DEBUG)
     # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        v = IliadConstraintsCostmapServer(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



