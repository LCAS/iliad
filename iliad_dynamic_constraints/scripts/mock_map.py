#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

Dummy OccupancyGrid creator to test feasible trajectory bending through iliad_smp_costmap:
    - This node creates a costmap with higher cost on the left of the robot.
    - We modify iliad_smp_costmap.yml to include it 
    - we should see that trajectories tend to happen on the left.

"""


import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose



class DummyCostmapServer(object):
    def __init__(self):
        rospy.loginfo("["+rospy.get_name()+"] " + "Starting ... ")

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        # ................................................................
        # other inits:

        self.ocGrid = OccupancyGrid()
        
        self.ocGrid.header.frame_id = self.base_frame_id
        self.ocGrid.info.resolution = self.grid_resolution
        self.ocGrid.info.height = self.grid_height
        self.ocGrid.info.width = self.grid_width

        # Centered around frame_id
        self.ocGrid.info.origin = Pose()        
        self.ocGrid.info.origin.position.x -= (self.grid_width/2.0) * self.grid_resolution 
        self.ocGrid.info.origin.position.y -= (self.grid_height/2.0)  * self.grid_resolution

        # Actual data inside the grid
        mat = np.full( (self.grid_height, self.grid_width), 0)
        mat[0:self.grid_height/2-10, 0:self.grid_width] = 100
        
        # y gradient
        x = np.linspace(0, self.grid_width*self.grid_resolution, self.grid_width)
        y = np.linspace(0,  self.grid_height*self.grid_resolution, self.grid_height)
        xv, mat = np.meshgrid(x, y)
        mat = (mat.max()-mat) / mat.max()
        mat = mat * mat *100

        self.ocGrid.data = mat.flatten(order='C')


        # ................................................................
        # Main loop
        rospy.loginfo("["+rospy.get_name()+"] " + "ROS init done.")
        while not rospy.is_shutdown():
            self.ocGrid.header.stamp = rospy.Time.now()
            self._map_pub.publish(self.ocGrid)
            self.rate.sleep()


    def initROS(self):
        # timers 
        self.rate = rospy.Rate(self.main_loop_freq) 

        # publishers
        self._map_pub = rospy.Publisher(self.vel_costmap_topic_name, OccupancyGrid, queue_size=10, latch=True)

        # service clients        
        # none here

        # subscribers and listeners
        # none here

        # service servers
        # none here

    # .............................................................................................................

    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)

        #  main loop update freq 1 hz
        self.main_loop_freq = rospy.get_param('~publish_freq', 1.0) 

        # Costmap topic
        self.vel_costmap_topic_name = rospy.get_param(
            '~vel_costmap_topic_name', '/robot'+str(self.robot_id)+'/dummy_costmap')

        # costmap data
        self.grid_resolution = rospy.get_param('~grid_resolution', 0.1)
        self.grid_height = rospy.get_param('~grid_height',300)  # resolution * height = y (12m.)
        self.grid_width = rospy.get_param('~grid_width',300) # resolution * width = x (20m.)

        # robot frame id
        self.base_frame_id = rospy.get_param(
            '~base_frame_id', '/robot'+str(self.robot_id)+'/base_link')


if __name__ == "__main__":
    rospy.init_node("dummy_costmap_node")
    v = DummyCostmapServer()
    rospy.spin()

