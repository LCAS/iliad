#!/usr/bin/env python

'''
Subscribes to  a bayes_people_tracker/PeopleTracker and republishes only specific tracks
'''

import rospy
from bayes_people_tracker.msg import PeopleTracker


class bayesRePublisher():

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

        self.selected_uuids = rospy.get_param('~uuids',  ['1', '2', '3', '4'])
        self.in_topic_name = rospy.get_param('~in_topic', '/robot5/people_tracker/positions_raw')
        self.out_topic_name = rospy.get_param('~out_topic', '/robot5/people_tracker/positions')

    def initROS(self):
        self.waitForTopic(self.in_topic_name)
        self.pub = rospy.Publisher(self.out_topic_name, PeopleTracker, queue_size=1)
        useEcho = (len(self.selected_uuids) == 0)
        if not useEcho:
            useEcho = (self.selected_uuids[0] == '')

        if not useEcho:
            rospy.Subscriber(self.in_topic_name, PeopleTracker, self.callback, queue_size=1)
        else:
            rospy.Subscriber(self.in_topic_name, PeopleTracker, self.echo_callback, queue_size=1)
            rospy.logwarn("[%s] Empty uuid list detected. Echoing data.", rospy.get_name())

    def echo_callback(self, msg):
        self.pub.publish(msg)

    def callback(self, msg):
        new_msg = PeopleTracker()
        new_msg.header = msg.header
        for i in range(len(msg.uuids)):
            if msg.uuids[i] in self.selected_uuids:
                new_msg.uuids.append(msg.uuids[i])
                new_msg.poses.append(msg.poses[i])
                new_msg.velocities.append(msg.velocities[i])
                new_msg.distances.append(msg.distances[i])
                new_msg.angles.append(msg.angles[i])
                if msg.distances[i] < new_msg.min_distance:
                    new_msg.min_distance = msg.distances[i]
                if msg.angles[i] < new_msg.min_distance_angle:
                    new_msg.min_distance_angle = msg.angles[i]
        self.pub.publish(new_msg)


# Main function.
if __name__ == '__main__':
    rospy.init_node('bayes_republisher_node')  # , log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = bayesRePublisher()
    except rospy.ROSInterruptException:
        pass
