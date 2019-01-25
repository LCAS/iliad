#!/usr/bin/env python

'''
Given a rosbag file, publishes as a goal the final pose the robot will go.
'''

import tf
from geometry_msgs.msg import PoseStamped
import rospy
import rosbag
from tf_bag import BagTfTransformer
import yaml


class pPublisher():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()
        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()
        # ................................................................
        # other params
        self.goalPoseSt = None
        # ................................................................
        rospy.loginfo("Node '%s' started.", rospy.get_name())
        rospy.loginfo("[%s]: Looking for tf between [%s] and [%s] in [%s]",
                      rospy.get_name(), self.robot_frame,
                      self.world_frame, self.bagFileName)
        self.getLastPose()

        if self.goalPoseSt == None:
            rospy.logerr("[%s]: GOAL NOT PUBLISHED!", rospy.get_name())
        else:
            rate = rospy.Rate(1)  # 1hz
            while not rospy.is_shutdown():
                self.goalPub.publish(self.goalPoseSt)
                rospy.logdebug("[%s]: Goal published in latched topic [%s]",
                               rospy.get_name(), self.goalTopicName)
                rate.sleep()

        rospy.spin()

    def loadROSParams(self):
        # Bag to analize
        self.bagFileName = rospy.get_param('~bagFilename', "/home/manolofc/iliad/tj/S1-T1.1-A1.bag")

        # robot frame
        self.robot_frame = rospy.get_param('~robot_frame_id', 'robot5/base_footprint')

        # world frame
        self.world_frame = rospy.get_param('~world_frame_id', 'world')

        # nav topic to pubish into
        self.goalTopicName = rospy.get_param('~goalTopic', '/robot5/move_base_simple/goal')

    def initROS(self):
        self.goalPub = rospy.Publisher(self.goalTopicName, PoseStamped, queue_size=1, latch=True)

    def getLastPose(self):
        rospy.loginfo("[%s]: Processing bag: " + self.bagFileName, rospy.get_name())

        bag = rosbag.Bag(self.bagFileName)
        info_dict = yaml.load(bag._get_yaml_info())
        endTime = rospy.Time.from_sec(info_dict['end'])
        startTime = rospy.Time.from_sec(info_dict['start'])
        bag_transformer = BagTfTransformer(bag)

        succeed = False
        while not succeed:
            try:
                if endTime <= startTime:
                    break
                translation, quaternion = bag_transformer.lookupTransform(
                    self.world_frame, self.robot_frame,  endTime)
                succeed = True
            except RuntimeError:
                endTime = endTime - rospy.Duration(0.1)
        if succeed:
            self.goalPoseSt = PoseStamped()
            self.goalPoseSt.header.frame_id = self.world_frame
            self.goalPoseSt.pose.position.x = translation[0]
            self.goalPoseSt.pose.position.y = translation[1]
            self.goalPoseSt.pose.position.z = translation[2]
            self.goalPoseSt.pose.orientation.x = quaternion[0]
            self.goalPoseSt.pose.orientation.y = quaternion[1]
            self.goalPoseSt.pose.orientation.z = quaternion[2]
            self.goalPoseSt.pose.orientation.w = quaternion[3]

        else:
            rospy.logerr("[%s]: Can't get a tf between [%s] and [%s] in [%s]",
                         rospy.get_name(), self.robot_frame,
                         self.world_frame, self.bagFileName)
        # close bag
        bag.close()


# Main function.
if __name__ == '__main__':
    rospy.init_node('lastRobotPose_node')  # , log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = pPublisher()
    except rospy.ROSInterruptException:
        pass
