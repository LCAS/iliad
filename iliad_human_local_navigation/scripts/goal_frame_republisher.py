#!/usr/bin/env python

'''
Subscribes to  a PoseStamped topic and republishes it in a diffent frame
'''

import tf
import rospy
from geometry_msgs.msg import PoseStamped


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

        self.out_frame_id = rospy.get_param('~out_frame_id', 'world')
        self.in_topic_name = rospy.get_param('~in_topic', '/gazebo/model_states')
        self.out_topic_name = rospy.get_param('~out_topic', '/actor_pose')

    def initROS(self):
        self.waitForTopic(self.in_topic_name)
        self.listener = tf.TransformListener()
        rospy.Subscriber(self.in_topic_name, PoseStamped, self.callback, queue_size=1)
        self.pub = rospy.Publisher(self.out_topic_name, PoseStamped, queue_size=1, latch=True)

    def callback(self, msg):
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform(
                self.out_frame_id, msg.header.frame_id, now, rospy.Duration(4.0))
            new_msg = self.listener.transformPose(self.out_frame_id, msg)
            self.pub.publish(new_msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception) as e:
            rospy.logerr("[%s] transform and publish failed (%s).", rospy.get_name(), e)


# Main function.
if __name__ == '__main__':
    rospy.init_node('model_pose_publisher_node')  # , log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = modelPublisher()
    except rospy.ROSInterruptException:
        pass
