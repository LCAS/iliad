#!/usr/bin/env python

'''
Publishes a fake trajectory  and report
'''

import rospy
from orunav_msgs.msg import ControllerTrajectoryChunkVec, ControllerReport, ControllerTrajectoryChunk, ControllerCommand, ControllerConstraints
from std_msgs.msg import Float64


class FakeTraj():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params
        r = rospy.Rate(0.5)  # 0.5 hz
        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [" + rospy.get_name() + "] STARTED")

        while not rospy.is_shutdown():
            self.trajectory = self.createTraj()
            self.report = self.createReport(self.trajectory)

            self.trajectory_pub.publish(self.trajectory)
            self.report_pub.publish(self.report)

            r.sleep()

    def createTraj(self):
        pass

    def createReport(self, traj):
        pass

    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 1)
        self.goal_frame_id = rospy.get_param('~goal_frame_id', 'world')

        self.trajectory_topic = rospy.get_param(
            '~trajectory_topic', '/robot'+str(self.robot_id)+'/control/controller/trajectories_mpc')

        self.reports_topic = rospy.get_param(
            '~reports_topic', '/robot' + str(self.robot_id) + '/control/controller/reports')

    def initROS(self):
        self.trajectory_pub = rospy.Publisher(
            self.trajectory_topic, ControllerTrajectoryChunkVec, queue_size=1)
        self.reports_pub = rospy.Publisher(self.reports_topic, ControllerReport, queue_size=1)

    # .............................................................................................................


# Main function.
if __name__ == '__main__':
    rospy.init_node('dynamic_constraints_node')  # , log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = DynamicConstraints()
    except rospy.ROSInterruptException:
        pass
