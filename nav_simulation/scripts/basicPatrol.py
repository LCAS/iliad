#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


'''

Simple script to test move base and vel parser.
Makes the robot move between two goals.


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
    rospy.loginfo("waiting move base!")
    client.wait_for_server()
    rospy.loginfo("Starting goal cycling")
    goal1 = getMeaGoal(0.04,-2.81)
    goal2 = getMeaGoal(-21.1,-1.0)

    goals = [goal1, goal2]
    i = 0
    while not rospy.is_shutdown():
        g = goals[i]
        i = (i + 1) % len(goals)

        g.target_pose.header.stamp = rospy.Time.now()
        rospy.loginfo("Going to "+str(g.target_pose.pose.position.x) + ", " +  str(g.target_pose.pose.position.y) ) 
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
