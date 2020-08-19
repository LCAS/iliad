#!/usr/bin/env python

# Based on publish_viapoints.py from christoph.roesmann@tu-dortmund.de
# does not work. it ends up in infinite looops

import rospy, math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import dynamic_reconfigure.client
import actionlib
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
import tf2_ros
import tf2_geometry_msgs

global tfBuffer, listener

def getRobotPoseSt():
    global tfBuffer, listener
    destFrame = "world"
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


def poseDist(poseStInit,poseStEnd):
    dist =  math.sqrt(math.pow(poseStEnd.pose.position.x - poseStInit.pose.position.x, 2) +
                      math.pow(poseStEnd.pose.position.y - poseStInit.pose.position.y, 2) +
                      math.pow(poseStEnd.pose.position.z - poseStInit.pose.position.z, 2))
    return dist

def isClose(goal):
    robotPoseSt = getRobotPoseSt()
    d = poseDist(goal,robotPoseSt)
    return (d<0.2)

def callback(config):
    pass
    #print(config)

def feedback_callback(config):
    pass
    #print(config)

def publish_via_points_msg():
  pub = rospy.Publisher('/actor00/move_base/TebLocalPlannerROS/via_points', Path, queue_size=1)
  goal_pub = rospy.Publisher('/actor00/move_base_simple/goal', PoseStamped, queue_size=1)
  move_base_client = actionlib.SimpleActionClient('/actor00/move_base',MoveBaseAction)
  move_base_client.wait_for_server()
  rospy.loginfo("["+rospy.get_name()+"] " + "Actionlib connected")

  via_points_msg = Path() 
  via_points_msg.header.stamp = rospy.Time.now()
  via_points_msg.header.frame_id = "world" # MFC: TEB always assumes world frame for waypoints
  
  p0=getRobotPoseSt()
  # Add some crude via-points
  x = [p0.pose.position.x,  -1.7,  -2.3,  -3.2,  -2.5,  -1.6, -0.0]
  y = [p0.pose.position.y, -17.3, -16.7, -15.8, -14.8, -13.9, -13.5]

  for i in range(1,len(x)):
        k = 4
        for j in range(0,k):
            point = PoseStamped()
            point.pose.position.x = ((k-j)*x[i] + j*x[i-1])/k
            point.pose.position.y = ((k-j)*y[i] + j*y[i-1])/k
            point.pose.orientation.w = 1
            point.header = via_points_msg.header
            via_points_msg.poses.append(point)

       
  pub.publish(via_points_msg)

  goal = MoveBaseGoal()
  goal.target_pose.pose.position.x = x[-1]
  goal.target_pose.pose.position.y = y[-1]
  goal.target_pose.pose.orientation.w = 1
  goal.target_pose.header = via_points_msg.header
  move_base_client.send_goal(goal,feedback_cb=feedback_callback)

# another way to send goal
#   point = PoseStamped()
#   point.pose.position.x = x[-1]
#   point.pose.position.y = y[-1]
#   point.pose.orientation.w = 1
#   point.header = via_points_msg.header
#   goal_pub.publish(point)

  hasArrived = False
  while (not hasArrived) and (not rospy.is_shutdown()):  
    # I shouldn't need doing this... but action service does not work as I thought
    # wait_time = 5
    # hasArrived = move_base_client.wait_for_result(rospy.Duration.from_sec( wait_time))
    
    hasArrived = isClose(goal.target_pose)

    if hasArrived:
        rospy.loginfo_throttle(5, "["+rospy.get_name()+"] " + "Goal reached. Clearing viapoints")
        via_points_msg.poses = []
        pub.publish(via_points_msg)
    else:
        rospy.loginfo_throttle(5, "["+rospy.get_name()+"] " + "Goal not reached yet ...")        
        status = move_base_client.get_state()
        rospy.loginfo_throttle(5, "["+rospy.get_name()+"] "+ "Base Status is: " + str(status) )

  via_points_msg.poses=[]
  pub.publish(via_points_msg)

if __name__ == '__main__': 
  global tfBuffer, listener
  try:
    rospy.init_node("test_via_points_msg")
    client = dynamic_reconfigure.client.Client("/actor00/move_base/TebLocalPlannerROS", timeout=30, config_callback=callback)
    # tf buffer for map to robot transforms
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # use provided viapoints, which are ordered, and take them seriously
    client.update_configuration({'global_plan_viapoint_sep': -1,'via_points_ordered':False, 'weight_viapoint':50})
    publish_via_points_msg()
  except rospy.ROSInterruptException:
    pass