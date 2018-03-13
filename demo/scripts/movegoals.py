#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


'''

Is /clock being published?
header: 
  seq: 2
  stamp: 
    secs: 523
    nsecs: 543000000
  frame_id: "world"
point: 
  x: -0.428505986929
  y: -1.70938253403
  z: 0.00372314453125
---
header: 
  seq: 3
  stamp: 
    secs: 523
    nsecs: 945000000
  frame_id: "world"
point: 
  x: -21.1201858521
  y: -0.980356752872
  z: -0.00018310546875



'''

def getMeaGoal(x,y):
    g = MoveBaseGoal()
    g.target_pose.header.frame_id = "world"
    g.target_pose.pose.position.x = x
    g.target_pose.pose.position.y = y
    g.target_pose.pose.orientation.w = 1.0
    return g



def movebase_client():

    client = actionlib.SimpleActionClient('/robot1/move_base',MoveBaseAction)
    client.wait_for_server()
    goal1 = getMeaGoal(-0.43,-1.71)
    goal2 = getMeaGoal(-21.1,-1.0)

    goals = [goal1, goal2]
    i = 0
    while not rospy.is_shutdown():
        g = goals[i]
        i = (i + 1) % len(goals)

        g.target_pose.header.stamp = rospy.Time.now()

        client.send_goal(g)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Can't get to:" + str(g))
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")

        



if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")