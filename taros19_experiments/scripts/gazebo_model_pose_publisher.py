#!/usr/bin/env python

'''
Subscribes to /gazebo/model_states, gets a model position and publishes into a Pose topic
'''

import tf
import rospy
import math
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates


class modelPublisher():

    def waitForTopic(self, topicFullName):
        isRunning = False

        while (not isRunning):
            currentTopics = rospy.get_published_topics()
            # flatten them
            currentTopics = [item for sublist in currentTopics for item in sublist]
            # print currentTopics
            isRunning = (topicFullName in currentTopics) or ('/'+topicFullName in currentTopics)
            if not isRunning:
                rospy.logerr('Topic '+topicFullName+' not found. Waiting 2 secs ...')
                rospy.sleep(2.)
        return

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()
        # ................................................................
        # Other config params

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("[%s] Node started.", rospy.get_name())

        rospy.spin()

    def loadROSParams(self):
        #
        self.map_frame = rospy.get_param('~map_frame', 'world')

        #
        self.model_name = rospy.get_param('~model_name', 'actor1')

        #
        self.gazebo_topic_name = rospy.get_param('~gazebo_models_topic', '/gazebo/model_states')
        #
        self.model_pose_topic_name = rospy.get_param('~model_pose_topic_name', '/actor_pose')

    def initROS(self):
        self.waitForTopic(self.gazebo_topic_name)
        rospy.Subscriber(self.gazebo_topic_name, ModelStates,
                         self.modelStatesCallback, queue_size=1)
        self.pub = rospy.Publisher(self.model_pose_topic_name, PoseStamped, queue_size=1)

    def modelStatesCallback(self, states):
        for i in range(len(states.name)):
            model_i = states.name[i]
            if model_i in self.model_name:
                #print model_i
                ans = PoseStamped()
                ans.header.stamp = rospy.Time.now()
                ans.header.frame_id = self.map_frame
                ans.pose = states.pose[i]
                self.pub.publish(ans)


# Main function.
if __name__ == '__main__':
    rospy.init_node('model_pose_publisher_node')  # , log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = modelPublisher()
    except rospy.ROSInterruptException:
        pass
