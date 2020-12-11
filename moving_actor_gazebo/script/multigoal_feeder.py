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

from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

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

        self.getTrajectories()
        # main loop here
        goOn = not rospy.is_shutdown()
        while goOn:
          for t in range(0,len(self.trajectories_data)):
            rospy.loginfo("Starting trajectory : "+str(t))
            (x,y,yaw) = self.getGoals(t)

            # teletransport the actor into the first position of the trajectory defined
            # calling gazebo service
            state_msg = ModelState()
            state_msg.model_name = self.model_name
            state_msg.pose.position.x = x[0]
            state_msg.pose.position.y = y[0]

            [qx,qy,qz,qw]=tf.transformations.quaternion_from_euler(0,0,yaw[0])
            state_msg.pose.orientation.x = qx
            state_msg.pose.orientation.y = qy
            state_msg.pose.orientation.z = qz
            state_msg.pose.orientation.w = qw
            resp = self.set_state( state_msg )

            # send them one by one
            for i in range(1,len(x)):          
              self.plotTargets(x[i:], y[i:], yaw[i:] )
              goal = self.createGoalMsg(x[i], y[i], yaw[i])
              self.move_base_client.send_goal(goal)
              
              finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(5)) 
            
              # If we don't get there in time, abort the goal
              if not finished_within_time:
                  self.move_base_client.cancel_goal()
                  rospy.loginfo("Timed out achieving goal")
              else:
                  # We made it!
                  state = self.move_base_client.get_state()
                  if state == 3:
                      rospy.loginfo("Goal succeeded!")

              #self.waitForArrival(i)

            # call service to send the actor to a position which doesn't bother when the traj is completed
            state_msg = ModelState()
            state_msg.model_name = self.model_name
            state_msg.pose.position.x = 20 + int(self.model_name[-1])
            state_msg.pose.position.y = 0
            resp = self.set_state( state_msg )

          goOn = (not rospy.is_shutdown()) and self.loop

          if goOn:
            rospy.loginfo("["+rospy.get_name()+"] " + "Re-starting trajectories")        
   
    # def waitForArrival(self, goal_i):      
    #   hasArrived = False
    #   fails = 0
    #   while (not hasArrived) and (not rospy.is_shutdown()):
    #     hasArrived = self.move_base_client.wait_for_result(rospy.Duration.from_sec(self.wait_time))              
    #     if hasArrived:
    #         rospy.logdebug_throttle(5, "["+rospy.get_name()+"] Intermediate goal " + str(goal_i)+" reached")
    #         fails = 0
    #     else:
    #         fails = fails + 1
    #         rospy.logdebug_throttle(5, "["+rospy.get_name()+"] " + "Goal not reached yet ... ("+ str(fails) + ")")        
    #         status = self.move_base_client.get_state()
    #         rospy.logdebug_throttle(5, "["+rospy.get_name()+"] "+ "Base Status is: " + str(status) )
    #         if fails ==( 30/self.wait_time): 
    #           rospy.logwarn( "["+rospy.get_name()+"] " + "Skipping goal")        
    #           hasArrived = True   
    #     print self.move_base_client.get_state() 
    #     rospy.sleep(0.1)  



    # def feedback_callback(self,config):
    #     #print(config)
    #     pass

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
        self.model_name = rospy.get_param('~model_name', "actor00")
        self.frame_id = rospy.get_param('~frame_id', "world")
        self.robot_base_frame_id = rospy.get_param('~robot_base_frame_id', "actor00/base_link")
        self.move_base_actionserver_name = rospy.get_param('~move_base_actionserver_name', '/actor00/move_base')
        self.trajectories_filename = rospy.get_param('~trajectories_filename', '../config/trajectories/trajectories.json')
        self.wait_time = rospy.get_param('~wait_time', 0.00005)
        self.interpolate = rospy.get_param('~interpolate', False)
        self.loop = rospy.get_param('~loop', True)
        self.visual_pub_topic_name = rospy.get_param('~visual_pub_topic_name', '/actor00/markers')
        
    def initROS(self):
        # publishers
        self.visual_pub = rospy.Publisher(self.visual_pub_topic_name, MarkerArray, queue_size=1, latch=True)

        # service clients         
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.loginfo("Gazebo set_model_state service available")

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

    def getGoals(self,trajectory_number):
      data = json.loads(self.trajectories_data[trajectory_number])
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

    def getTrajectories(self):
      with open(self.trajectories_filename) as traj_file:
        self.trajectories_data = np.genfromtxt(self.trajectories_filename,dtype="string",delimiter="\n")     

        #rospy.loginfo(self.trajectories_data)
        #rospy.loginfo(self.trajectories_number)

    
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


if __name__ == '__main__': 
  rospy.init_node("multigoal_provider")#, log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
  try:
      v = GoalFeeder(rospy.get_name())
  except rospy.ROSInterruptException:
      pass