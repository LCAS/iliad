#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 30 16:07:34 2015

@author: cdondrup
"""

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, PoseStamped, Vector3
import numpy as np
#import time
from threading import Lock


class CostmapCreator(object):
    # Function look-up table to create the right angle and min and max speed
    # for the given qtc state
    _calc_pre_sets_functions = {
        # Undefined state
        '?':  lambda a,v,mi,ma: (a,(mi,ma)),
        '??': lambda a,v,mi,ma: (a,(mi,ma)),
        #QTCb
        '-': lambda a,v,mi,ma: (a,(mi,ma)),
        '+': lambda a,v,mi,ma: (a + np.pi if a < 0.0 else a - np.pi,(mi,ma)),
        '0': lambda a,v,mi,ma: (
                np.arctan2(v.y, v.x),
                (
                    int(np.sqrt(np.power(v.x,2)+np.power(v.y,2))*100)-5 if int(np.sqrt(np.power(v.x,2)+np.power(v.y,2))*100)-5 > mi else mi,
                    int(np.sqrt(np.power(v.x,2)+np.power(v.y,2))*100)+5 if int(np.sqrt(np.power(v.x,2)+np.power(v.y,2))*100)+5 < ma else ma
                )
            ),
        # QTCc
        '-,-': lambda a,v,mi,ma: (a + np.pi/4,(mi,ma)),
        '-,+': lambda a,v,mi,ma: (a - np.pi/4,(mi,ma)),
        '-,0': lambda a,v,mi,ma: (a,(mi,ma)),
        '+,-': lambda a,v,mi,ma: (a + 3*np.pi/4,(mi,ma)),
        '+,+': lambda a,v,mi,ma: (a - 3*np.pi/4,(mi,ma)),
        '+,0': lambda a,v,mi,ma: (a + np.pi if a < 0.0 else a - np.pi,(mi,ma)),
        '0,-': lambda a,v,mi,ma: (a + np.pi/2,(mi,ma)),
        '0,+': lambda a,v,mi,ma: (a - np.pi/2,(mi,ma)),
        '0,0': lambda a,v,mi,ma: (a,(0,0))
    }

    # Look-up table for the size of the low cost are
    _size_lookup_table = {
        # Undefined state
        '?':  2*np.pi,
        '?,?': 2*np.pi,
        # QTCb
        '-': np.pi,
        '+': np.pi,
        '0': np.pi/32,
        #QTCc
        '-,-': np.pi/2,
        '-,+': np.pi/2,
        '-,0': np.pi/16,
        '+,-': np.pi/2,
        '+,+': np.pi/2,
        '+,0': np.pi/16,
        '0,-': np.pi/16,
        '0,+': np.pi/16,
        '0,0': 0.0
    }

    def __init__(self, map_pub, origin_pub, width=100, height=100, max_costs=100, min_costs=0, resolution=0.05):
        self._max_costs = max_costs
        self._min_costs = min_costs
        self._resolution = resolution
        self._map_pub = map_pub
        self._origin_pub = origin_pub

        try:
            self.move_base_topic = rospy.get_param("~move_base_topic")
        except KeyError as e:
            rospy.logerr("[" + rospy.get_name()+"] " + "Unable to get parameter: " + str(e))

        local_planner_name = self.get_local_planner_name()
        self._max_vel_x_parma_name = self.move_base_topic + "/" + local_planner_name + "/max_vel_x"

        self.lock = Lock()
        
    def get_local_planner_name(self):
        try:
            return rospy.get_param(self.move_base_topic + "/" + "base_local_planner").split('/')[1]
        except KeyError as e:
            rospy.logerr("[" + rospy.get_name()+"] " + "Unable to get parameter: " + str(e))
            rospy.logerr("Is move_base running? Will retry in 5 second.")
            rospy.sleep(5.)
            if not rospy.is_shutdown(): return self.get_local_planner_name()

    @property
    def max_costs(self):
        return self._max_costs

    @max_costs.setter
    def max_costs(self, max_costs):
        with self.lock:
            self._max_costs = max_costs

    @property
    def min_costs(self):
        return self._min_costs

    @min_costs.setter
    def min_costs(self, min_costs):
        with self.lock:
            self._min_costs = min_costs

    @property
    def resolution(self):
        return self._resolution

    @resolution.setter
    def resolution(self, resolution):
        with self.lock:
            self._resolution = resolution

    def _create_costmap(self, angle=0.0, velocity=Vector3(), qtc_symbol='0', size=100, min_speed=0, max_speed=50, min_cost=0):
        cost_array = None

        cost_array = self._fast_costmap_creator(
            pre_sets=self._calc_pre_sets_functions[qtc_symbol](angle, velocity, min_speed, max_speed),
            qtc_symbol=qtc_symbol,
            map_size=size,
            min_cost=min_cost,
            cost_array=cost_array
        )

        # Transpose due to shit interpretation of the array by ROS
        return cost_array.T

    def _fast_costmap_creator(self, pre_sets=(0.0, (np.nan, np.nan)), qtc_symbol="0", map_size=100, min_cost=0, cost_array=None):
#        start_loop = time.time()
        # Sign flip
        pre_sets = (np.pi - (np.abs(pre_sets[0])-np.pi), pre_sets[1]) if pre_sets[0] < -np.pi else pre_sets
        pre_sets = (-np.pi + (np.abs(pre_sets[0])-np.pi), pre_sets[1]) if pre_sets[0] > np.pi else pre_sets
        size = self._size_lookup_table[qtc_symbol]
        len_qtc_symbol = len(qtc_symbol.split(','))

        if cost_array == None:
            cost_array = np.empty((map_size, map_size))
            cost_array.fill(self.max_costs)

        if not qtc_symbol == "0,0": # 00 only needs full costs everywhere
            cp = self._cartesian_product( # Cartesian product of x and y indices
                [
                    np.arange(-int(np.floor(float(map_size)/2)), int(np.ceil(float(map_size)/2)), step=1),
                    np.arange(-int(np.floor(float(map_size)/2)), int(np.ceil(float(map_size)/2)), step=1)
                ]
            )
            polar = self._cartesian_to_polar(cp[:,0],cp[:,1]) # Create polar for every combination of x and y
            min_cost += 15 if not qtc_symbol in ('?','?,?') and len_qtc_symbol == 2 else 0
            #The loop could maybe vectorised
            for s in reversed(np.arange(size/4,size+0.01,size/4)) if not qtc_symbol in ('?','?,?') and len_qtc_symbol == 2 else [size]:
                idx = np.logical_and( # Find polar inside specified range
                    np.logical_and( # Min and max speed = min and max distance from robot
                        polar[0] <= pre_sets[1][1],
                        polar[0] >= pre_sets[1][0]
                    ),
                    np.logical_and( # Find angles according to desired shape
                        np.logical_or(
                            polar[1] >= -(s/2) + pre_sets[0],
                            polar[1] <= -(np.pi - ((np.abs(pre_sets[0])+(s/2))-np.pi)) # Sign flip in free area
                        ),
                        np.logical_or(
                            polar[1] <= (s/2) + pre_sets[0],
                            polar[1] >= np.pi - ((np.abs(pre_sets[0])+(s/2))-np.pi) # Sign flip in free area
                        )
                    )
                ).reshape(map_size, map_size)
                cost_array[idx] = min_cost
                min_cost -= 5 if not qtc_symbol in ('?','?,?') and len_qtc_symbol == 2 else 0
        cost_array[
            int(np.floor(float(map_size)/2))-1:int(np.ceil(float(map_size)/2))+1,
            int(np.floor(float(map_size)/2))-1:int(np.ceil(float(map_size)/2))+1
        ] = self.min_costs # (0,0) should never have costs
#        print "elapsed:", time.time() - start_loop
        return cost_array


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

    def publish(self, angle, qtc_symbol, velocity):
        with self.lock: # Making sure no dynamic variable is changed during calculation
            try:
                max_speed = int(rospy.get_param(self._max_vel_x_parma_name)*100) # Magic number: 100 =  make int
            except KeyError as e:
                rospy.logwarn("No such parameter: " + e.message)
                return
            size = max_speed*2 # Magic number: 2 = double the size to have max_vel_x in all directions
            o = OccupancyGrid()
            o.header.stamp = rospy.Time.now()
            o.header.frame_id = 'base_link'
            o.info.resolution = self.resolution
            o.info.height = size
            o.info.width = size
            o.info.origin = Pose()
            o.info.origin.position.x -= (o.info.width/2) * o.info.resolution # Translate to be centered under robot
            o.info.origin.position.y -= (o.info.height/2)  * o.info.resolution
            p = PoseStamped(header=o.header, pose=o.info.origin)
            self._origin_pub.publish(p)

            o.data = self._create_costmap(
                angle=angle,
                qtc_symbol=qtc_symbol,
                size=size,
                velocity=velocity,
                max_speed=max_speed,
                min_cost=self.min_costs
            ).flatten(order='C')
            self._map_pub.publish(o)

    def _cartesian_to_polar(self, x, y):
        rho = np.sqrt(np.power(x,2) + np.power(y,2))
        phi = np.arctan2(y, x)
        return(rho, phi)

    def _polar_to_cartesian(self, rho, phi):
        x = np.multiply(rho, np.cos(phi))
        y = np.multiply(rho, np.sin(phi))
        return(x.astype(int), y.astype(int))
