#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
HEAVILY BASED ON cdondrup's costmap_creator from Strands project


"""


import rospy
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer, Cache
from bayes_people_tracker.msg import PeopleTracker
#from iliad_velocity_costmaps.iliad_costmap_creator import IliadCostmapCreator
from std_msgs.msg import String

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from dynamic_reconfigure.server import Server as DynServer
from iliad_velocity_costmaps.cfg import VelocityCostmapsConfig
from visualization_msgs.msg import Marker
from tf import TransformListener
import tf2_geometry_msgs
import tf2_ros
import json
from std_msgs.msg import Float64MultiArray

from geometry_msgs.msg import Pose, Vector3
#import time
from threading import Lock


class IliadCostmapCreator(object):

    def __init__(self, base_link_tf, map_pub, origin_pub, qtc_rules, width=100, height=100,resolution=0.05):
        self._resolution = resolution
        self._map_pub = map_pub
        self._origin_pub = origin_pub
        self.qtc_rules_d = qtc_rules

        self.base_link_tf = base_link_tf
        self.size_x = width # max_speed*2
        self.size_y = height # max_speed*2
        # Magic number: 2 = double the size to have max_vel_x in all directions

        o = OccupancyGrid()
        o.header.stamp = rospy.Time.now()
        o.header.frame_id = self.base_link_tf
        o.info.resolution = self.resolution
        o.info.height = self.size_x
        o.info.width = self.size_y
        o.info.origin = Pose()
        o.info.origin.position.x -= (o.info.width/2.0) * o.info.resolution # Translate to be centered under robot
        o.info.origin.position.y -= (o.info.height/2.0)  * o.info.resolution
        o.data = np.full( (self.size_x, self.size_y), 0).T.flatten(order='C')
     
        self. current_occ_grid = o
        self.lock = Lock()
        
    @property
    def resolution(self):
        return self._resolution

    @resolution.setter
    def resolution(self, resolution):
        with self.lock:
            self._resolution = resolution

    def qtc2pennalty(self, qtc_data):
        c_p = 0
        a_p = 0
        angle = 0

        qtc_data = qtc_data.split(',')
        human_h = qtc_data[0]
        human_v = qtc_data[2]
        robot_h = qtc_data[1]
        robot_v = qtc_data[3]
         
        try:
            (c_p, a_p, angle) = self.qtc_rules_d[''.join(qtc_data)]
        except KeyError:
            pass

        rospy.logerr("Node [" + rospy.get_name() + "] " + "I got \n\t- qtc: [" + ''.join(qtc_data) + "]\n"+
        "\t- human_h: [" + human_h + "]\n" +
        "\t- human_v: [" + human_v + "]\n" +
        "\t- robot_h: [" + robot_h + "]\n" +
        "\t- robot_v: [" + robot_v + "]\n" +
        "....................................\n" +
        "\t- weight_p: [" + str(c_p) + "]\n" +
        "\t- weight_a: [" + str(a_p) + "]\n" +
        "\t- ang_inc: [" + str( round(angle*180.0/np.pi,2) ) + "]\n" +
        "\n")
        return (c_p, a_p, angle)

    def _create_costmap(self, angle=0.0, velocity=Vector3(), qtc_symbol='0', size_x=100, size_y=100 ):
        cost_array = None
        cost_array = self._fast_costmap_creator(
            angle=angle, 
            velocity=velocity,
            qtc_symbol=qtc_symbol,
            size_x=size_x,
            size_y=size_y,
            cost_array=cost_array
        )
        self.curr_grid = cost_array       
        # Transpose due to shit interpretation of the array by ROS
        return cost_array.T

    def _fast_costmap_creator(self, angle=0.0, velocity=(0,0,0), qtc_symbol="0", size_x=100, size_y=100, cost_array=None):
        (connect_penalty, ang_pennalty, ang_inc) = self.qtc2pennalty(qtc_symbol.data)

        center_angle = angle + ang_inc/4.0

        
        # Sign flip
        if center_angle < -np.pi:    
                center_angle = np.pi - (np.abs(center_angle)-np.pi)
        if center_angle > np.pi:    
                center_angle = -np.pi + (np.abs(center_angle)-np.pi)
        
        # first, fill all with min costs == 0
        cost_array = np.empty((size_x, size_y))
        cost_array.fill(0)

        # second get polar indexes
        cp = self._cartesian_product( # Cartesian product of x and y indices
            [
                np.arange(-int(np.floor(float(size_x)/2)), int(np.ceil(float(size_x)/2)), step=1),
                np.arange(-int(np.floor(float(size_y)/2)), int(np.ceil(float(size_y)/2)), step=1)
            ]
        )
        polar = self._cartesian_to_polar(cp[:,0],cp[:,1]) # Create polar for every combination of x and y
        

        cost_array = self.setcost(cost_array, center_angle, np.abs(ang_inc), connect_penalty, polar)
        cost_array = self.setcost(cost_array, center_angle+ang_inc, np.abs(ang_inc), ang_pennalty, polar)
        #cost_array = self.setcost(cost_array, center_angle-ang_inc, np.abs(ang_inc), ang_pennalty, polar)

        return cost_array

    def setcost(self, carray, centre, arc, val,polar):

        min_speed =-1000
        max_speed =1000

        idx = np.logical_and( # Find polar inside specified range
            np.logical_and( # Min and max speed = min and max distance from robot
                polar[0] <= max_speed,
                polar[0] >= min_speed
            ),
            np.logical_and( # Find angles according to desired shape
                np.logical_or(
                    polar[1] >= -(arc/2) + centre,
                    polar[1] <= -(np.pi - ((np.abs(centre)+(arc/2))-np.pi)) # Sign flip in free area
                ),
                np.logical_or(
                    polar[1] <= (arc/2) + centre,
                    polar[1] >= np.pi - ((np.abs(centre)+(arc/2))-np.pi) # Sign flip in free area
                )
            )
        ).reshape(carray.shape[0], carray.shape[1])
        carray[idx] = val
        return carray


    def _cartesian_product(self, arrays, out=None):
        """
        Generate a cartesian product of input arrays.

        Parameters
        ----------
        arrays : list of array-like
            1-D arrays to form the cartesian product of.
        out : ndarray
            Array to place the cartesian product in.

        Returns
        -------
        out : ndarray
            2-D array of shape (M, len(arrays)) containing cartesian products
            formed of input arrays.

        Examples
        --------
        >>> cartesian(([1, 2, 3], [4, 5], [6, 7]))
        array([[1, 4, 6],
               [1, 4, 7],
               [1, 5, 6],
               [1, 5, 7],
               [2, 4, 6],
               [2, 4, 7],
               [2, 5, 6],
               [2, 5, 7],
               [3, 4, 6],
               [3, 4, 7],
               [3, 5, 6],
               [3, 5, 7]])

        """

        arrays = [np.asarray(x) for x in arrays]
        dtype = arrays[0].dtype

        n = np.prod([x.size for x in arrays])
        if out is None:
            out = np.zeros([n, len(arrays)], dtype=dtype)

        m = n / arrays[0].size
        out[:,0] = np.repeat(arrays[0], m)
        if arrays[1:]:
            self._cartesian_product(arrays[1:], out=out[0:m,1:])
            for j in xrange(1, arrays[0].size):
                out[j*m:(j+1)*m,1:] = out[0:m,1:]
        return out

    def update_map(self, angle, velocity, qtc_symbol):

        with self.lock: # Making sure no dynamic variable is changed during calculation

            o = OccupancyGrid()
            o.header.stamp = rospy.Time.now()
            o.header.frame_id = self.base_link_tf
            o.info.resolution = self.resolution
            o.info.height = self.size_x
            o.info.width = self.size_y
            o.info.origin = Pose()
            o.info.origin.position.x -= (o.info.width/2) * o.info.resolution # Translate to be centered under robot
            o.info.origin.position.y -= (o.info.height/2)  * o.info.resolution
            p = PoseStamped(header=o.header, pose=o.info.origin)
            self._origin_pub.publish(p)

            o.data = self._create_costmap(
                angle=angle,
                qtc_symbol=qtc_symbol,
                velocity=velocity,
                size_x = self.size_x,
                size_y = self.size_y
            ).flatten(order='C')
            self._map_pub.publish(o)
            self. current_occ_grid = o
    def _cartesian_to_polar(self, x, y):
        rho = np.sqrt(np.power(x,2) + np.power(y,2))
        phi = np.arctan2(y, x)
        return(rho, phi)

    def _polar_to_cartesian(self, rho, phi):
        x = np.multiply(rho, np.cos(phi))
        y = np.multiply(rho, np.sin(phi))
        return(x.astype(int), y.astype(int))

    def pose2cell(self,x,y):
        # map 0,0 meters is robot pose...
        i = j = 0
        isValid = True
        i = int((x  / self.resolution) - (self.size_x/2.0) )
        j =  int((y - (self.size_y/2.0) ) / self.resolution)

        if i > (self.size_x-1)  or i<0 or  j > (self.size_y-1)  or j<0:
            isValid = False
         
        return (i,j,isValid)


    def valueAtPose(self, px, py):
        value = -1
        isInside = False

        (ci, cj, isInside) = self.pose2cell(px,py)
        if isInside:
            # T.flatten(order='C')
            value = self.curr_grid[ci,cj]
        (value, isInside)

class IliadVelocityCostmapServer(object):
    def __init__(self, name):
        rospy.loginfo("["+rospy.get_name()+"] " + "Starting ... ")

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        # other params
        qtc_rules = json.load(open(self.qtc_rules_file))
        self.icc = IliadCostmapCreator(self.base_frame_id, self.vel_costmap_topic_pub, self.origin_topic_pub,  qtc_rules )
        self.dyn_srv = DynServer(VelocityCostmapsConfig, self.dyn_callback)
        
        rospy.loginfo("["+rospy.get_name()+"] " + "... all done.")

    def initROS(self):
        # publishers
        self.vel_costmap_topic_pub = rospy.Publisher(self.vel_costmap_topic_name, OccupancyGrid, queue_size=10, latch=True)
        self.origin_topic_pub = rospy.Publisher(self.origin_topic_name, PoseStamped, queue_size=10)
        self.vis_marker_topic_pub = rospy.Publisher(self.vis_marker_topic_name, Marker, queue_size=1)
        self.vel_constraints_topic_pub = rospy.Publisher(self.vel_constraints_topic_name, Float64MultiArray, queue_size=1)
        # service clients
        
        # none here

        # subscribers and listeners
        self.qtc_state_topic_sub = Subscriber(self.qtc_state_topic_name, String)
        self.qtc_cache = Cache(self.qtc_state_topic_sub, 10, allow_headerless=True)
        self.human_tracking_topic_sub = Subscriber(self.human_tracking_topic_name, PeopleTracker)

        subs = [ self.qtc_state_topic_sub, self.human_tracking_topic_sub ]
        self.ts = ApproximateTimeSynchronizer( subs, 60, slop=0.1 , allow_headerless=True)
        self.ts.registerCallback(self.callback)
        
        self.listenerBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.listenerBuffer)
        
        # service servers
        # none here

    # .............................................................................................................

    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)

        # MARKER for visual data
        self.vis_marker_topic_name = rospy.get_param(
            '~tracked_person_vis_topic_name', '/robot'+str(self.robot_id)+'/tracked_person_vis_topic')

        # costmap origin
        self.origin_topic_name = rospy.get_param("~origin_topic_name", '/robot'+str(self.robot_id)+"/origin")

        # Costmap topic
        self.vel_costmap_topic_name = rospy.get_param(
            '~vel_costmap_topic_name', '/robot'+str(self.robot_id)+'/qsr/velocity_costmap')

        # robot frame id
        self.base_frame_id = rospy.get_param(
            '~base_frame_id', '/robot'+str(self.robot_id)+'/base_link')
        # in tf2, frames do not have the initial slash
        if (self.base_frame_id[0] == '/'):
            self.base_frame_id = self.base_frame_id[1:]

        # human tracking trackedpersons topic
        self.human_tracking_topic_name = rospy.get_param(
            '~human_tracking_topic_name', '/robot'+str(self.robot_id)+'/qsr/people_tracker/positions')

        # qtc detections topic
        self.qtc_state_topic_name = rospy.get_param(
            '~qtc_state_topic_name', '/robot'+str(self.robot_id)+'/qtc_state_topics')
        
        # this hardcoded should never be used ...
        self.qtc_rules_file = rospy.get_param(
            '~qtc_rules_file', '/home/manolofc/workspace/ms3/src/iliad_velocity_costmaps/cfg/rules_qtc.yaml')
        
        # tranform tf_timeout
        timeout = rospy.get_param('~tf_timeout', 2)
        self.listener_timeout = rospy.Duration(timeout)

        # speed constraints: TODO get from planner.
        self.min_rotat_vel = rospy.get_param('~min_rotat_vel', 0.01 )
        self.max_rotat_vel = rospy.get_param('~max_rotat_vel', 0.5 )
        self.min_trans_vel = rospy.get_param('~min_trans_vel', 0.01 )
        self.max_rotat_vel = rospy.get_param('~max_rotat_vel', 1.0)
        self.vel_constraints_topic_name = rospy.get_param('~vel_constraints_topic', '/robot'+str(self.robot_id)+'/velocity_constraints')
 

     
    def dyn_callback(self, config, level):
        self.icc.resolution = config["costmap_resolution"]
        return config

    def callback(self, qtc, ppl):
        min_index = ppl.distances.index(ppl.min_distance)
        
        min_pose = ppl.poses[min_index]
        min_ang = ppl.angles[min_index]
        min_vel = ppl.velocities[min_index]

        self.publish_closest_person_marker(min_pose, ppl.header.frame_id)

        self.icc.update_map(min_ang, min_vel,qtc)
        
        # test get cell values ...
        for next_x in [-1,0,1]:
            for next_y in [-1,0,1]:
                (cx,cy,basur) = self.icc.pose2cell(next_x, next_y)
                test_val = self.icc.valueAtPose(next_x, next_y)
                self.icc
                rospy.logerr("Node [" + rospy.get_name() + "] " + "Pose [" + str(next_x) + ", " + str(next_y)+ "] == Cell [" + str(cx) + ", " + str(cy)+ "]  [" + str(test_val) + "]\n")


    def publish_closest_person_marker(self, pose, frame_id):
        if self.vis_marker_topic_pub.get_num_connections() > 0:
            m = Marker()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = frame_id
            m.ns = "velocity_costmaps"
            m.id = 0
            m.type = m.SPHERE
            m.action = m.MODIFY
            m.pose = pose
            m.pose.position.z = 2.0
            m.scale.x = .25
            m.scale.y = .25
            m.scale.z = .25
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.lifetime = rospy.Duration(1.)
            self.vis_marker_topic_pub.publish(m)


    def update_vel_constraints(self, next_x, next_y):
        constraint_v = self.max_trans_vel
        constraint_w = self.max_rotat_vel

        # get cost of cell  where next robot pose is going to be
        (cost, isInside) = self.icc.valueAtPose(next_x, next_y)

        if isInside:
            # cost is a percent ...
            constraint_v =  self.min_trans_vel + (self.max_trans_vel - self.min_trans_vel ) * float(100 - cost)/100.0
            constraint_w =  self.min_rotat_vel + (self.max_rotat_vel - self.min_rotat_vel ) * float(100 - cost)/100.0

        self.pub_vel_constraints(constraint_v, constraint_w)

    def pub_vel_constraints(self,constraint_v, constraint_w):
        msg = Float64MultiArray()
        msg.data.append(constraint_v)
        msg.data.append(constraint_w)
        self.vel_constraints_topic_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("velocity_costmap_server")
    v = IliadVelocityCostmapServer(rospy.get_name())
    rospy.spin()

