#!/usr/bin/env python

'''

Reads a Twist intended for a differential drive and delivers same for a cart
It's almost this one
http://wiki.ros.org/teb_local_planner/Tutorials/Planning%20for%20car-like%20robots
'''

import tf
import rospy, math
from geometry_msgs.msg import Twist

def convert_trans_rot_vel_to_steering_angle(vtrans, distWheels, vrot, backWheelsRadius, carLen):
    
    v = vtrans 
    phi =math.atan2(vrot * backWheelsRadius * carLen,vtrans * distWheels)
    
    return (v,phi)

def getYaw():
    global tfListener
    robotTFName ='/robot1/base_link'
    wheelTFName = '/robot1/sd_wheel_link'
    
    wheelTFName ='/robot1/base_link'
    robotTFName = '/robot1/sd_wheel_link'

    gotOne = False
    while not gotOne:
        
        try:
            anyTime = rospy.Time(0)
            tfListener.waitForTransform(robotTFName,wheelTFName,  anyTime, rospy.Duration(0))
            rel_pose, rel_quat = tfListener.lookupTransform(robotTFName,wheelTFName,anyTime)
            (rel_rol, rel_pitch, rel_yaw) = tf.transformations.euler_from_quaternion(rel_quat)
            
            gotOne = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.TransformException) as e:
            rospy.logerr(str(e))
            
    rel_yaw_swift =(math.pi - rel_yaw) # so that 0 points ahead
    rospy.loginfo("Curr yaw is %f", float(rel_yaw_swift))
    
    return rel_yaw_swift


def cmd_callback(data):
  global distBtBackWheels
  global backWheelsRadius
  global wheelsAxesDist
  global pub
   
  (v,steering) = convert_trans_rot_vel_to_steering_angle(data.linear.x, distBtBackWheels, data.angular.z, backWheelsRadius, wheelsAxesDist)
  msg = Twist()
  msg.linear.x = v
  msg.angular.z = 0.0
  
  pub.publish(msg)
  

if __name__ == '__main__': 
  try:
    
    rospy.init_node('cmd_vel_to_ackermann_drive_2')      
    twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
    ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
    distBtBackWheels = rospy.get_param('~distBtBackWheels', 1.0)
    backWheelsRadius = rospy.get_param('~backWheelsRadius', 0.05)
    wheelsAxesDist = rospy.get_param('~wheelsAxesDist', 1.5)  
    
    tfListener = tf.TransformListener()

    rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
    pub = rospy.Publisher(ackermann_cmd_topic, Twist, queue_size=1)
    
    rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s.", twist_cmd_topic, ackermann_cmd_topic)
    
    rospy.spin()
    
  except rospy.ROSInterruptException:
    pass
