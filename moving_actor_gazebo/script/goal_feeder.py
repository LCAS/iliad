#!/usr/bin/env python

# Based on publish_viapoints.py from christoph.roesmann@tu-dortmund.de
# does not work. it ends up in infinite looops

import rospy, math
from geometry_msgs.msg import PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
import tf
import tf2_ros
import tf2_geometry_msgs



def poseDist(poseStInit,poseStEnd):
    dist =  math.sqrt(math.pow(poseStEnd.pose.position.x - poseStInit.pose.position.x, 2) +
                      math.pow(poseStEnd.pose.position.y - poseStInit.pose.position.y, 2) +
                      math.pow(poseStEnd.pose.position.z - poseStInit.pose.position.z, 2))
    return dist

def isClose(goal):
    robotPoseSt = getRobotPoseSt()
    d = poseDist(goal,robotPoseSt)
    return (d<0.2)

def feedback_callback(config):
    pass
    #print(config)

# ......................................................

global tfBuffer, listener

def getRobotPoseSt(destFrame):
    global tfBuffer, listener
    inPoseStamped = PoseStamped()
    inPoseStamped.pose.orientation.w = 1
    inPoseStamped.header.stamp = rospy.Time.now()
    inPoseStamped.header.frame_id = "actor00/base_link"

    absPoseStamped = None
    t0 = rospy.Time()
    try:
        trans = tfBuffer.lookup_transform(destFrame, inPoseStamped.header.frame_id, t0, rospy.Duration(5.0))
        absPoseStamped = tf2_geometry_msgs.do_transform_pose(inPoseStamped, trans)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("TF between %s and %s not ready: [%s]" % (destFrame, inPoseStamped.header.frame_id, str(e)))

    return absPoseStamped

def publish_goals():
  move_base_client = actionlib.SimpleActionClient('/actor00/move_base',MoveBaseAction)
  move_base_client.wait_for_server()
  rospy.loginfo("["+rospy.get_name()+"] " + "Actionlib connected")


  frame_id = "world"
  p0=getRobotPoseSt(frame_id )
  # Add some crude goals
  x = [p0.pose.position.x,  -1.7,  -2.3,  -3.2,  -2.5,  -1.6, -0.0]
  y = [p0.pose.position.y, -17.3, -16.7, -15.8, -14.8, -13.9, -13.5]  
   
  wait_time = 0.001
  for i in range(1,len(x)):
        k = 4
        for j in range(0,k):     
            p0=getRobotPoseSt(frame_id)       
            goal = MoveBaseGoal()
            goal.target_pose.pose.position.x = (j*x[i] + (k-j)*x[i-1])/k
            goal.target_pose.pose.position.y = (j*y[i] + (k-j)*y[i-1])/k

            yaw = math.atan2(goal.target_pose.pose.position.y - p0.pose.position.y, goal.target_pose.pose.position.x - p0.pose.position.x)
            new_quaternion = tf.transformations.quaternion_from_euler(0,0, yaw)
            goal.target_pose.pose.orientation.x = new_quaternion[0]
            goal.target_pose.pose.orientation.y = new_quaternion[1]
            goal.target_pose.pose.orientation.z = new_quaternion[2]
            goal.target_pose.pose.orientation.w = new_quaternion[3]
            
            if i==1:
              goal.target_pose.pose.orientation = p0.pose.orientation

            goal.target_pose.header.frame_id = frame_id
            goal.target_pose.header.stamp = rospy.Time.now()
            move_base_client.send_goal(goal,feedback_cb=feedback_callback)

            hasArrived = False
            fails = 0
            while (not hasArrived) and (not rospy.is_shutdown()):  
              # I shouldn't need doing this... but action service does not work as I thought              
              hasArrived = move_base_client.wait_for_result(rospy.Duration.from_sec(wait_time))              
              #hasArrived = isClose(goal.target_pose)

              if hasArrived:
                  rospy.loginfo_throttle(5, "["+rospy.get_name()+"] Intermediate goal " + str(4*j+i)+" reached")
                  fails = 0
              else:
                  fails = fails + 1
                  rospy.loginfo_throttle(5, "["+rospy.get_name()+"] " + "Goal not reached yet ... ("+ str(fails) + ")")        
                  status = move_base_client.get_state()
                  rospy.loginfo_throttle(5, "["+rospy.get_name()+"] "+ "Base Status is: " + str(status) )
                  if fails ==( 30/wait_time): 
                    rospy.loginfo( "["+rospy.get_name()+"] " + "Skipping goal")        
                    hasArrived = True

if __name__ == '__main__': 
  global tfBuffer, listener
  try:
    rospy.init_node("goal_provider")
    # tf buffer for map to robot transforms
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    publish_goals()
  except rospy.ROSInterruptException:
    pass