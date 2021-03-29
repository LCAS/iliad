#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

This version creates a costmap with same size, resolution and frame id than static navigation map.
It also publishes constraint costmap as an update, keeping most of the (potentially huge) costmap unchanged.

"""

import rospy
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from message_filters import Subscriber
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Pose, PointStamped, PoseStamped, Quaternion

from bayes_people_tracker.msg import PeopleTracker

import numpy as np
from threading import Lock
#from scipy.sparse import *
from matplotlib.path import Path


class ConstraintsCostmapV2(object):
    """
        Here lies the costmap logic.
    """

    def __init__(self, ref_costmap, update_width, update_height, d2, d1, w, blind_offset ):        
        # Robot shape: This is a rectangle, but we can use other stuff 
        self.d2 = d2  # robot center to fork edge distance
        self.d1 = d1  # robot center to front laser distance
        self.w = w # truck width
        self.blind_offset  = blind_offset # extra margin                
        self.robotVerts = [(self.d1+self.blind_offset, self.w/2+self.blind_offset), (-self.d2-self.blind_offset, self.w/2+self.blind_offset), (-self.d2-self.blind_offset, -self.w/2-self.blind_offset), (self.d1+self.blind_offset, -self.w/2-self.blind_offset)]


        self.current_occ_grid = ref_costmap
        self.current_occ_grid.header.seq = -1

        # these are for easiness...
        self.resolution = ref_costmap.info.resolution
        self.height = ref_costmap.info.height
        self.width  = ref_costmap.info.width 
        self.ox = self.current_occ_grid.info.origin.position.x 
        self.oy = self.current_occ_grid.info.origin.position.y 

        #MFC: layered costmap kind of assumes same position and orientation in all static maps... so DON'T use rotations!
        # self.oa = self.get_rotation(self.current_occ_grid.info.origin.orientation)

        # MFC: odd stuff happens if you use different static map sizes...
        # self.height = int(20.0 / self.resolution) #y
        # self.width  = int(26.0 / self.resolution) #x
        # self.current_occ_grid.info.height = self.height
        # self.current_occ_grid.info.width = self.width

        # Translate to be centered under costmap_frame_id
        #MFC: layered costmap kind of assumes same position and orientation in all static maps... so DON'T use rotations!
        # self.current_occ_grid.info.origin = Pose()        
        # self.current_occ_grid.info.origin.position.x = -(self.width/2.0)   * self.resolution 
        # self.current_occ_grid.info.origin.position.y = -(self.height/2.0)  * self.resolution


        # clear data
        self.current_occ_grid.data = np.zeros(self.width * self.height)
        self.lock = Lock()

        self.local_human_pose = None
        self.local_robot_pose = None
        self.situation = None
        self.angle_bounds = np.array([0, 2.0*np.pi])
        # Updates will be our way of adding new data
        self.update = OccupancyGridUpdate()
        self.update.header = self.current_occ_grid.header
        self.update.header.seq = -1

        #sergi
        
        self.update.width = self.width
        self.update.height = self.height
        self.update.x = 0
        self.update.y = 0
        self.update.data = np.zeros(self.update.width * self.update.height)

    def get_map(self):
        with self.lock: 
                now = rospy.Time.now()
                # ..........................................
                self.current_occ_grid.header.stamp = rospy.Time.now()
                self.current_occ_grid.header.seq = self.current_occ_grid.header.seq + 1
                # ..........................................
                dur = rospy.Time.now() - now
                rospy.logdebug("Node [" + rospy.get_name() + "] " +
                                "Init Map created in (" + str(dur.to_sec()) + ") secs"
                                )      
                return self.current_occ_grid
    
    def update_human(self, humanPoseSt):
        with self.lock: 
            if not (humanPoseSt == None):
                self.local_human_pose = humanPoseSt            
                #self.printPoseSt(humanPoseSt, "human pose received")

    def update_robot(self, robotPoseSt):
        with self.lock: 
            if not (robotPoseSt == None):
                self.local_robot_pose = robotPoseSt            
                #self.printPoseSt(robotPoseSt, "robot pose received")
                                
                update_center_x = robotPoseSt.pose.position.x
                update_center_y = robotPoseSt.pose.position.y

                # get center cell
                (ih0,jh0,valid) = self.pose2cell(update_center_x, update_center_y)
                #rospy.logdebug("Node [" + rospy.get_name() + "] " + "Update center at (" + str(ih0) + ", " + str(jh0) + ") " ) 
                #rospy.logdebug("Node [" + rospy.get_name() + "] " + "Update dims  are (" + str(self.update.width) + ", " + str(self.update.height) + ") " ) 

                # get update margins
                minx = ih0 - self.update.width/2
                miny = jh0 - self.update.height/2
                maxx = ih0 + self.update.width/2
                maxy = jh0 + self.update.height/2
                #rospy.logdebug("Node [" + rospy.get_name() + "] " + "BB is (" + str(minx) + ", " + str(miny) + ") to (" + str(maxx) + ", " +str(maxy) + ")" ) 

                # adjust update center to fit inside grid
                if minx<0:
                    ih0 = self.update.width/2
                elif maxx > self.width:
                    ih0 = self.width - self.update.width/2
                if miny<0:
                    jh0 = self.update.height/2
                elif maxy > self.height:
                    jh0 = self.height - self.update.height/2

                #rospy.logdebug("Node [" + rospy.get_name() + "] " + "New update center at (" + str(ih0) + ", " + str(jh0) + ") " )                                 
                (update_center_x, update_center_y,valid) = self.cell2pose(ih0,jh0)

                # update grid starts here                
                self.update.x = 0#ih0 - self.update.width/2
                self.update.y = 0#jh0 - self.update.height/2    

                # grids
                self.min_update_x = update_center_x -(self.update.width/2.0)* self.resolution 
                self.max_update_x = update_center_x + (self.update.width /2.0)* self.resolution
                self.min_update_y = update_center_y -(self.update.height/2.0)* self.resolution 
                self.max_update_y = update_center_y + (self.update.height/2.0)* self.resolution

                x = np.linspace(self.min_update_x, self.max_update_x, self.update.width)        
                y = np.linspace(self.min_update_y, self.max_update_y , self.update.height)        
                self.xx, self.yy = np.meshgrid(x, y)   
                xg, yg = self.xx.flatten(), self.yy.flatten()
                self.grid_points = np.vstack((xg,yg)).T 

                # initial clear space
                #self.update.data = np.zeros(self.update.width * self.update.height)


    def update_angle_bounds(self, angle_bounds):
        with self.lock: 
                self.angle_bounds = angle_bounds            

    def update_situation(self, situation):
        with self.lock: 
                self.situation = situation 

    def getRobotPolyAt(self,x,y,a):
        # translate and rotate to given reference        
        newVerts = []
        for xi,yi in self.robotVerts:
            nx =  (xi * np.cos(a)) - (yi * np.sin(a)) + x
            ny =  (xi * np.sin(a)) + (yi * np.cos(a)) + y
            newVerts.append((nx,ny))

        # make a polygon
        robot_polygon = Path(newVerts) 
        return robot_polygon 


    def pose2cell(self,x,y):
        ix, iy = self.getRel(x,y)
        i = int((ix / self.resolution))   
        j = int((iy / self.resolution))  

        isValid = self.isInside(i,j)
         
        return (i,j,isValid)
    
    def getRel(self,x,y):
        # MFC: layered costmap kind of assumes same position and orientation in all static maps... so DON'T rotate!
        dx = (x - self.ox)
        dy = (y - self.oy)
        # nx = dx * np.cos(self.oa) + dy * np.sin(self.oa) 
        # ny = -dx * np.sin(self.oa) + dy * np.cos(self.oa) 
        return (dx,dy)

    def getAbs(self,dx,dy):
        # MFC: layered costmap kind of assumes same position and orientation in all static maps... so DON'T rotate!
        x = (dx + self.ox)
        y = (dy + self.oy)
        # x = x * np.cos(self.oa) - y * np.sin(self.oa) 
        # y = x * np.sin(self.oa) + y * np.cos(self.oa) 
        return (x,y)        

    def cell2pose(self,i,j):     
        dx = i  * self.resolution
        dy = j  * self.resolution
        (x,y) = self.getAbs(dx,dy)

        isValid = self.isInside(i,j)

        return (x,y,isValid)

    def isInside(self, i,j):
        return self.isInside0(i,j,self.width,self.height)

    def isInside0(self, i,j,w,h):
        isValid = True
        if (i > (w-1))  or (i<0) or (j > (h-1))  or (j<0):
            isValid = False
        return isValid
   
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

    def get_map_update(self):
        #rospy.loginfo("["+rospy.get_name()+"] " + "Situation: " + str(self.situation))
        # timestamping ...
        self.update.header.seq = self.update.header.seq + 1
        self.update.header.stamp = rospy.Time.now()
        with self.lock: 
            if (not self.local_human_pose == None) and (not self.local_robot_pose == None) and (rospy.get_param('~use_qhrsi_planning', 'false')):
                now = rospy.Time.now()
                # ............................

                # get human pose and cell
                xh = self.local_human_pose.pose.position.x 
                yh = self.local_human_pose.pose.position.y
                ah = self.get_rotation(self.local_human_pose.pose.orientation)
                # there is a lot of error in estimations of human orientation. This may be noisy ...!

                #(ih,jh,valid) = self.pose2cell(xh,yh)

                # get robot position
                xr = self.local_robot_pose.pose.position.x 
                yr = self.local_robot_pose.pose.position.y
                ar = self.get_rotation(self.local_robot_pose.pose.orientation)                
                
                # clear "canvas"
                self.update.data = np.zeros(self.update.width * self.update.height)

                # this is human cell pose, relative to update grid
                # ih = ih - self.update.x 
                # jh = jh - self.update.y 

                # this creates a square around human
                # for i in range(0,update.width):
                #     for j in range(0,update.height):                        
                #         k = self.linIndex0(i,j, update.width)
                #         if (abs(ih-i)<int(update.width/20)) and (abs(jh-j)<int(update.height/20)):
                #             update.data[k] = 100

                # distance to human in each cell
                
                dh = np.sqrt(np.power(self.xx-xh,2)+np.power(self.yy-yh,2))
                
                
                # distance to robotin each cell
                dr = np.sqrt(np.power(self.xx-xr,2)+np.power(self.yy-yr,2))

                # angle between human and any point
                ahp = np.arctan2(self.yy - yh, self.xx - xh)
                # # angle between robot and any point
                # arp = np.arctan2(yy-yr,xx-xr)
                # # angle between human and robot
                ahr = np.arctan2(yh-yr,xh-xr)
                # # angle between human robot and any point
                # arg = (arp - ahr)
                # # using that aproach we had some rounding errors
                # # this is the same as above, but avoiding computing two arctans
                (a1, a2) = (self.yy-yr,self.xx-xr)
                (b1, b2)  = (yh-yr,xh-xr)                
                arg = np.arctan2(  a2*b1 - a1*b2, a1*b1+a2*b2 )
                
                # angle cost
                ca = np.exp(-np.power(4*arg,2))
                
                # distance cost
                cd = np.exp(-np.power(0.75*(dh),2))
                
                # remap costs
                ca = np.interp(ca, (ca.min(), ca.max()), (0, 1))
                cd = np.interp(cd, (cd.min(), cd.max()), (0, 1))

                # combine costs
                c_no_sit = ca * cd

                # remap 0-100
                c_no_sit = np.interp(c_no_sit, (c_no_sit.min(), c_no_sit.max()), (0, 100))

                if self.situation == None:
                    c = c_no_sit
                else:
                    if self.situation in ["PBL", "ROL", "PCL"]:
                        # allow left side of human ...
                        lower_angle_lim = 0
                        upper_angle_lim = np.pi
                    elif self.situation in ["PBR", "ROR", "PCR"]:
                        # allow right side of human ...
                        lower_angle_lim = np.pi
                        upper_angle_lim = 2.0*np.pi

                    # human orientation is in -pi,pi range...
                    angle_rel = np.mod(ahp  - np.mod(ahr , 2*np.pi), 2*np.pi )
                    c =  c_no_sit
                    forbid_indexes = (angle_rel <= lower_angle_lim ) | (angle_rel >= upper_angle_lim)
                    #c[ forbid_indexes ] = 100
                    
                    # cd_strong = np.exp(-np.power(0.01*(dh),2))
                    cd_strong = np.exp(-np.power(0.3*(dh),2))
                    cd_strong = np.interp(cd_strong, (cd_strong.min(), cd_strong.max()), (0, 100 ))
                    c[ forbid_indexes ] =  cd_strong[forbid_indexes] 

                # avoid overlapping costs over the robot
                robot_polygon = self.getRobotPolyAt(xr,yr,ar)
                grid = robot_polygon.contains_points(self.grid_points)
                # now you have a mask with points inside a polygon
                mask = grid.reshape(self.update.height,self.update.width)         
                c[mask] = 0

                self.update.data =  c.flatten(order='C')


                # mark margins
                # self.update.data[0] = 100
                # self.update.data[update.width*update.height-1] = 100
                                            
                # ..........................................
                dur = rospy.Time.now() - now
                rospy.logdebug_throttle(5,"Node [" + rospy.get_name() + "] " +
                                "Map update built in (" + str(dur.to_sec()) + ") secs"
                                )
            else:
                self.update.data = np.zeros(self.update.width * self.update.height)


        # always send update, even if it's emtpy    
        return self.update

    def printPoseSt(self, poseSt,text):
        rospy.loginfo("Node [" + rospy.get_name() + "] " +
                text + ": Pose ( " +
                '{0:.2f}'.format(poseSt.pose.position.x) + ", " + '{0:.2f}'.format(poseSt.pose.position.y) + ", " +
                '{0:.2f}'.format(self.get_rotation(poseSt.pose.orientation) * 180.0 /np.pi) + " deg) " +
                " in frame (" + poseSt.header.frame_id+ ")"
                )      

    def get_rotation (self,orientation_q):
        (roll, pitch, yaw) = euler_from_quaternion ([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        return yaw
              

class IliadConstraintsCostmapServerV2(object):

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

        rospy.Subscriber(self.situation_topic_name, String, self.sit_callback, queue_size=1)

        rospy.Subscriber(self.angle_bounds_topic_name, Float64MultiArray, self.angle_bounds_callback, queue_size=1)

        self.listenerBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.listenerBuffer)
        
        # service servers

        # ... none here

        # Timers
        rospy.Timer(rospy.Duration(self.update_publish_period), self.update_map, oneshot=False)


    def angle_bounds_callback(self, msg):
        coefs = msg.data
        if len(coefs) == 2:
            if (float(coefs[0]) < float(coefs[1])):
                self.angle_bounds = np.array(coefs) * np.pi/180
                try:
                    self.icc.update_angle_bounds(self.angle_bounds)
                except AttributeError as ae:
                    # first msg may arrive even before creating the map....
                    pass
                                
            else:
                rospy.logerr("Node [" + rospy.get_name() + "] Lower angle bound bigger than higher one. (" + str(
                    coefs[0])+") >= (" + str(coefs[1])+") ")
        else:
            rospy.logerr("Node [" + rospy.get_name() +
                         "] More than 2 tangential vel bounds provided. (" + str(len(coefs))+") ")

    def sit_callback(self, sit_string):
      
        if str(sit_string.data) in ["PBL", "PBR", "ROL", "ROR", "PCL", "PCR", "DEBUG"]:
            #rospy.loginfo("["+rospy.get_name()+"] " + "Situation: " + str(sit_string.data))
            self.situation = str(sit_string.data)
        else:
            self.situation = None
            rospy.logwarn("["+rospy.get_name()+"] " + "Situation: Unmodelled")

        try:
            self.icc.update_situation(self.situation)
        except AttributeError as ae:
            # first msg may arrive even before creating the map....
            pass
        
    
    def map_callback(self, map):
        self.map = map
        if not self.gotMap:
            rospy.loginfo("["+rospy.get_name()+"] " + "Map received. Creating costmap.")
            self.icc = ConstraintsCostmapV2(self.map, self.update_width, self.update_height, self.d2, self.d1, self.w, self.blind_offset )
            rospy.loginfo("["+rospy.get_name()+"] " + "Costmap created. Publishing initial Map.")
            self.costmap_topic_pub.publish(self.icc.get_map())
        self.gotMap = True

    # .............................................................................................................
    def update_map(self, event):
        if self.gotMap:
            self.costmap_updates_topic_pub.publish(self.icc.get_map_update())

    def loadROSParams(self):

        self.robot_id = rospy.get_param('~robot_id', 4)

        # Output Costmap topic
        self.costmap_topic_name = rospy.get_param(
            '~costmap_topic_name', '/robot'+str(self.robot_id)+'/qsr/constraints_costmap')

        # angle bounds for debug
        self.angle_bounds_topic_name = rospy.get_param(
            '~angle_bounds_topic_name', '/robot'+str(self.robot_id)+'/angle_bounds')

        # Input map topic
        self.map_topic_name = rospy.get_param(
            '~map_topic_name', '/map_laser2d')

        # human tracking trackedpersons (SPENCER) topic
        self.human_tracking_topic_name = rospy.get_param(
            '~human_tracking_topic_name', '/robot'+str(self.robot_id)+'/qsr/people_tracker/positions')
         
        # current robot position from tf tree
        self.robot_pose_topic_name = rospy.get_param(
            '~robot_pose_topic_name', '/robot' + str(self.robot_id) + '/robot_poseST')

        # current human robot interaction from Laurences node
        self.situation_topic_name = rospy.get_param(
            '~situation_topic_name', '/robot' + str(self.robot_id) + '/qsr/situation_predictions')

        # costmap update configuration.
        self.update_center_x = rospy.get_param('~update_center_x', 15.3)
        self.update_center_y = rospy.get_param('~update_center_y', -3.03)
        self.update_width  = rospy.get_param('~update_width', 375)
        self.update_height = rospy.get_param('~update_height', 306)
        self.update_publish_period = rospy.get_param('~update_publish_period', 0.05)

        self.d2 = rospy.get_param('~d2',0.3)  # robot center to fork edge distance
        self.d1 = rospy.get_param('~d1',1.5)  # robot center to front laser distance
        self.w = rospy.get_param('~w',0.8) # truck width
        self.blind_offset = rospy.get_param('~blind_offset',0.20) # extra margin        


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
    rospy.init_node("constraints_costmap_server_v2")# , log_level=rospy.DEBUG)
     # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        v = IliadConstraintsCostmapServerV2(rospy.get_name())
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



