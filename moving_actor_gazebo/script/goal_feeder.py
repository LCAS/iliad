#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
goal feeder: read goals from a json file and feeds them to a move_base action service
"""

import math
import json
import os
import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
import tf
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import MarkerArray, Marker



class GoalFeeder(object):

    def __init__(self, name):
        rospy.loginfo("["+rospy.get_name()+"] " + "Starting ... ")

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # other params

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("["+rospy.get_name()+"] " + "Init done.")

        # main loop here
        goOn = not rospy.is_shutdown()
        while goOn:
          (x,y,yaw) = self.getGoals(self.goals_filename)

          # send them one by one
          for i in range(0,len(x)):          
            self.plotTargets(x[i:], y[i:], yaw[i:] )
            goal = self.createGoalMsg(x[i], y[i], yaw[i])
            self.move_base_client.send_goal(goal,feedback_cb=self.feedback_callback)
            self.waitForArrival(i)

          goOn = (not rospy.is_shutdown()) and self.loop
          if goOn:
            rospy.loginfo("["+rospy.get_name()+"] " + "Re-starting goal sending")        
   
    def waitForArrival(self, goal_i):      
      hasArrived = False
      fails = 0
      while (not hasArrived) and (not rospy.is_shutdown()):
        hasArrived = self.move_base_client.wait_for_result(rospy.Duration.from_sec(self.wait_time))              

        if hasArrived:
            rospy.logdebug_throttle(5, "["+rospy.get_name()+"] Intermediate goal " + str(goal_i)+" reached")
            fails = 0
        else:
            fails = fails + 1
            rospy.logdebug_throttle(5, "["+rospy.get_name()+"] " + "Goal not reached yet ... ("+ str(fails) + ")")        
            status = self.move_base_client.get_state()
            rospy.logdebug_throttle(5, "["+rospy.get_name()+"] "+ "Base Status is: " + str(status) )
            if fails ==( 30/self.wait_time): 
              rospy.logwarn( "["+rospy.get_name()+"] " + "Skipping goal")        
              hasArrived = True      


    def createGoalMsg(self,x,y,a):    
      goal = MoveBaseGoal()
      goal.target_pose.pose.position.x = x
      goal.target_pose.pose.position.y = y
      new_quaternion = tf.transformations.quaternion_from_euler(0,0, a)
      goal.target_pose.pose.orientation.x = new_quaternion[0]
      goal.target_pose.pose.orientation.y = new_quaternion[1]
      goal.target_pose.pose.orientation.z = new_quaternion[2]
      goal.target_pose.pose.orientation.w = new_quaternion[3]

      goal.target_pose.header.frame_id = self.frame_id
      goal.target_pose.header.stamp = rospy.Time.now()
      return goal

    def plotTargets(self, x,y,a):
        data = MarkerArray()

        for i in range(0,len(x)):
          (xi, yi, ai) = (x[i], y[i], a[i])
          arrow = self.getArrowMarker(i, xi, yi, ai)
          data.markers.append(arrow)

        # Finally publish .......................
        self.visual_pub.publish(data)

    def getArrowMarker(self,i, xi, yi, ai):
        arrow = Marker()
        arrow.id = i
        arrow.type = Marker.ARROW
        arrow.header.frame_id = self.frame_id

        arrow.header.stamp = rospy.Time.now()
        arrow.ns = "goals"
        arrow.action = Marker.ADD
        
        arrow.pose.position.x = xi
        arrow.pose.position.y = yi
        new_quaternion = tf.transformations.quaternion_from_euler(0,0, ai)
        arrow.pose.orientation.x = new_quaternion[0]
        arrow.pose.orientation.y = new_quaternion[1]
        arrow.pose.orientation.z = new_quaternion[2]
        arrow.pose.orientation.w = new_quaternion[3]

        arrow.scale.x = 2*0.15
        arrow.scale.y = 2*0.025
        arrow.scale.z = 2*0.025

        # color
        arrow.color.r = 0
        arrow.color.g = 0
        arrow.color.b = 1
        arrow.color.a = 1.0
        return arrow

    def loadROSParams(self):
        self.frame_id = rospy.get_param('~frame_id', "world")
        self.robot_base_frame_id = rospy.get_param('~robot_base_frame_id', "actor00/base_link")

        self.move_base_actionserver_name = rospy.get_param(
            '~move_base_actionserver_name', '/actor00/move_base')

        self.goals_filename = rospy.get_param(
            '~goals_filename', '../config/trajectories/trajectory.json')

        self.wait_time = rospy.get_param(
            '~wait_time', 0.00005)

        self.interpolate = rospy.get_param(
            '~interpolate', False)

        self.loop = rospy.get_param(
            '~loop', True)

        self.visual_pub_topic_name = rospy.get_param(
            '~visual_pub_topic_name', '/actor00/markers')
        
    def initROS(self):
        # publishers
        self.visual_pub = rospy.Publisher(self.visual_pub_topic_name, MarkerArray, queue_size=1, latch=True)

        # service clients        
        # ... none here
        # action service clients        
        self.move_base_client = actionlib.SimpleActionClient(self.move_base_actionserver_name,MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo("["+rospy.get_name()+"] " + "Actionlib connected")

        # subscribers and listeners
        # tf buffer for map to robot transforms
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # service servers
        # ... none here
        # Timers
        # ... none here

        # Others        

    def getGoals(self,filename):
      with open(filename) as json_file:
        data = json.load(json_file)
        xgl = np.array(data['x'])
        ygl = np.array(data['y'])

        if 'yaw' in data.keys():
          agl = data['yaw'] 
        else:
          p0=self.getRobotPoseSt(self.frame_id)
          xgl = np.insert(xgl,0,p0.pose.position.x)
          ygl = np.insert(ygl,0,p0.pose.position.y)
          agl = np.arctan2(np.diff(ygl), np.diff(xgl))
          xgl = xgl[1:]
          ygl = ygl[1:]

        if self.interpolate:
          (xgl,ygl,agl) = self.interpolateGoals(xgl,ygl,agl)
          
      return (xgl,ygl,agl)
    
    def getRobotPoseSt(self, destFrame):

        inPoseStamped = PoseStamped()
        inPoseStamped.pose.orientation.w = 1
        inPoseStamped.header.stamp = rospy.Time.now()
        inPoseStamped.header.frame_id = self.robot_base_frame_id

        absPoseStamped = None
        t0 = rospy.Time()
        try:
            trans = self.tfBuffer.lookup_transform(destFrame, inPoseStamped.header.frame_id, t0, rospy.Duration(5.0))
            absPoseStamped = tf2_geometry_msgs.do_transform_pose(inPoseStamped, trans)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF between %s and %s not ready: [%s]" % (destFrame, inPoseStamped.header.frame_id, str(e)))

        return absPoseStamped

    def interpolateGoals(self, x,y,a):
      p0=self.getRobotPoseSt(self.frame_id)
      x = np.insert(x,0,p0.pose.position.x)
      y = np.insert(y,0,p0.pose.position.y)
      k = 4
      xn = []
      yn = [] 

      # quick and dirty interp ...
      for i in range(1,len(x)):
        k = 4
        for j in range(0,k):     
            xij = (j*x[i] + (k-j)*x[i-1])/k
            yij = (j*y[i] + (k-j)*y[i-1])/k
            xn.append(xij)
            yn.append(yij)
      an = np.arctan2(np.diff(yn), np.diff(xn))
      xn = xn[1:]
      yn = yn[1:]
      return (xn, yn, an)

    def feedback_callback(self,config):
        pass
        #print(config)

    # not used
    def poseDist(self, poseStInit,poseStEnd):
        dist =  math.sqrt(math.pow(poseStEnd.pose.position.x - poseStInit.pose.position.x, 2) +
                          math.pow(poseStEnd.pose.position.y - poseStInit.pose.position.y, 2) +
                          math.pow(poseStEnd.pose.position.z - poseStInit.pose.position.z, 2))
        return dist

    # not used
    def isClose(self, goal):
        robotPoseSt = self.getRobotPoseSt()
        d = self.poseDist(goal,robotPoseSt)
        return (d<0.2)

if __name__ == '__main__': 
  rospy.init_node("goal_provider")#, log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
  try:
      v = GoalFeeder(rospy.get_name())
  except rospy.ROSInterruptException:
      pass