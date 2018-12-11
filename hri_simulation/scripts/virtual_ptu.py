#!/usr/bin/env python

# code from scosar@enrichme

import rospy
import sensor_msgs
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import datetime

rospy.init_node('virtual_ptu')
rate=rospy.Rate(15)

ptu_topic = rospy.get_param('~ptu_topic', '/ptu/state') 
s = rospy.Publisher(ptu_topic, JointState, queue_size=100)

while not rospy.is_shutdown():
    j = JointState()
    j.header = Header()
    #j.header.stamp = {'secs' : datetime.datetime.now().time().second , 'nsecs' : datetime.datetime.now().time().microsecond}
    now = rospy.Time.now()
    j.header.stamp.secs = now.secs
    j.header.stamp.nsecs = now.nsecs
    j.header.frame_id = '/base_link'
    j.name = ['tilt','pan']
    j.position = [0,0]
    j.velocity = [0,0]
    j.effort = [0,0]
    s.publish(j)
    rate.sleep()
#print j



rospy.spin()