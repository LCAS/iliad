#!/usr/bin/env python

'''
Connects to trajectory topic and report
and publishes next goal in robot plan (geometry_msgs::PoseStamped)


MPC controller receives the following from VEN
1. a command activating a trajectory (2) with id_i
2. a command starting tracking (3) with time t_i
3. a trajectory (id_i)

And MPC keeps sending to VEN (and coordinator)
- reports, with chunk and step of trajectory id_i just completed

DynamicConstraints intercepts commands and trajectory and sends

'''

import rospy
from geometry_msgs.msg import PoseStamped
from orunav_msgs.msg import ControllerTrajectoryChunkVec, ControllerReport, ControllerTrajectoryChunk, ControllerCommand, ControllerConstraints
from std_msgs.msg import Float64MultiArray
import tf
from threading import Lock


class DynamicConstraints():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params
        self.commands = []
        self.local_traj_id = 0
        self.curr_chunk = -1

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [" + rospy.get_name() + "] STARTED")

        rospy.spin()

    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)
        self.goal_frame_id = rospy.get_param('~goal_frame_id', 'world')

        self.tangential_velocity_topic = rospy.get_param(
            '~bounds_tangential_velocity_topic', '/robot'+str(self.robot_id)+'/control/controller/bounds_tangential_velocity')

        self.steering_velocity_topic = rospy.get_param(
            '~bounds_steering_velocity_topic', '/robot'+str(self.robot_id)+'/control/controller/bounds_steering_velocity')

        self.spatial_coefs_topic = rospy.get_param(
            '~spatial_coefs_topic', '/robot'+str(self.robot_id)+'/control/controller/spatial_coefs')

        self.in_command_topic = rospy.get_param(
            '~in_command_topic', '/robot'+str(self.robot_id)+'/control/controller/commands')

        self.out_command_topic = rospy.get_param(
            '~out_command_topic', '/robot'+str(self.robot_id)+'/control/controller/commands_mpc')

        self.in_trajectory_topic = rospy.get_param(
            '~in_trajectory_topic', '/robot'+str(self.robot_id)+'/control/controller/trajectories')

        self.out_trajectory_topic = rospy.get_param(
            '~out_trajectory_topic', '/robot'+str(self.robot_id)+'/control/controller/trajectories_mpc')

        self.in_reports_topic = rospy.get_param(
            '~reports_topic', '/robot' + str(self.robot_id) + '/control/controller/reports_mpc')
        self.out_reports_topic = rospy.get_param(
            '~reports_topic', '/robot' + str(self.robot_id) + '/control/controller/reports')

        isPositionConstraintEnabled = rospy.get_param(
            '/robot'+str(self.robot_id)+'/params/controller/enable_position_constraints')
        if not isPositionConstraintEnabled:
            rospy.logerr("Node [" + rospy.get_name() +
                         "] Position constraints disabled. Enabling them in this node ")
            rospy.set_param('/robot'+str(self.robot_id) +
                            '/params/controller/enable_position_constraints', True)

    def initROS(self):
        self.trajectory_pub = rospy.Publisher(
            self.out_trajectory_topic, ControllerTrajectoryChunkVec, queue_size=1)
        self.command_pub = rospy.Publisher(self.out_command_topic, ControllerCommand, queue_size=1)
        self.reports_pub = rospy.Publisher(self.out_reports_topic, ControllerReport, queue_size=1)

        rospy.Subscriber(self.in_trajectory_topic, ControllerTrajectoryChunkVec,
                         self.trajectory_callback, queue_size=1)
        rospy.Subscriber(self.in_command_topic, ControllerCommand,
                         self.command_callback, queue_size=1)
        rospy.Subscriber(self.in_reports_topic, ControllerReport,
                         self.reports_callback, queue_size=1)

        rospy.Subscriber(self.tangential_velocity_topic, Float64MultiArray,
                         self.bounds_tangential_velocity_callback, queue_size=1)
        rospy.Subscriber(self.steering_velocity_topic, Float64MultiArray,
                         self.bounds_steering_velocity_callback, queue_size=1)
        rospy.Subscriber(self.spatial_coefs_topic, Float64MultiArray,
                         self.spatial_coefs_callback, queue_size=1)
    # .............................................................................................................

    def bounds_tangential_velocity_callback(self, msg):
        coefs = msg.data
        if len(coefs) == 2:
            if coefs[0] < coefs[1]:
                self.bounds_tangential_velocity = coefs
            else:
                rospy.logerr("Node [" + rospy.get_name() + "] Lower tangential velocity bound bigger than higher one. " + str(
                    coefs[0])+"> " + str(coefs[1])+" ")
        else:
            rospy.logerr("Node [" + rospy.get_name() +
                         "] More than 2 tangential vel bounds provided. (" + str(len(coefs))+") ")

    def bounds_steering_velocity_callback(self, msg):
        coefs = msg.data
        if len(coefs) == 2:
            if coefs[0] < coefs[1]:
                self.steering_velocity = coefs
            else:
                rospy.logerr("Node [" + rospy.get_name() + "] Lower steering velocity bound bigger than higher one. " + str(
                    coefs[0])+"> " + str(coefs[1])+" ")
        else:
            rospy.logerr("Node [" + rospy.get_name() +
                         "] More than 2 steering vel bounds provided. (" + str(len(coefs))+") ")

    def spatial_coefs_callback(self, msg):
        coefs = msg.data
        l_coefs = len(coefs)
        if (l_coefs % 3) == 0:
            self.spatial_coef_a0 = coefs[0:l_coefs/3]
            self.spatial_coef_a1 = coefs[(l_coefs/3):(2*l_coefs/3)]
            self.spatial_coef_b = coefs[(2*l_coefs/3):]
        else:
            rospy.logerr("Node [" + rospy.get_name() +
                         "] Spatial coefs can't be split into three same-lenght vectors. Len is (" + str(len(coefs))+") ")

    '''
    Reports are continously sent. They contain current controller status and
    car state.
    We will use them to trigger sucesive local trajectory transmissions
    '''

    def reports_callback(self, msg):

        has_sent = False

        # you need it to be idle. Using CONTROLLER_STATUS_FINALIZE (finishing) didn't work well.
        if (msg.status == ControllerReport.CONTROLLER_STATUS_WAIT):
            has_sent = self.sendNext()

        # if it didn't transmit more trajectories
        if not has_sent:
            # We need to report in terms of the current reference trajectory,
            # chunk and step, not the local ones.
            msg = self.translateChunks(msg)
            # and send them
            self.reports_pub.publish(msg)

    def translateChunks(self, msg):
        # what we just completed ...
        local_chunk_ind = msg.traj_chunk_sequence_num
        local_step_ind = msg.traj_step_sequence_num
        if len(msg.traj_values) > 0:
            local_traj_id = msg.traj_values[0].traj_id

        # what we are telling VEN we just did
        global_chunk_ind = max(self.curr_chunk - 1, 0)
        global_step_ind = msg.traj_step_sequence_num
        if len(msg.traj_values) > 0:
            global_traj_id = self.trajectories.chunks[0].traj_id

        # repopulate msg.
        # finished transmitting...
        if (self.curr_chunk == -1):
            msg.traj_chunk_sequence_num = 0
            msg.traj_step_sequence_num = 0
            msg.traj_values = []
        else:
            msg.traj_chunk_sequence_num = int(global_chunk_ind)
            msg.traj_step_sequence_num = int(global_step_ind)
            if len(msg.traj_values) > 0:
                msg.traj_values[0].traj_id = global_traj_id
        return msg

    def command_callback(self, msg):
        rospy.loginfo("Node [" + rospy.get_name() + "] command (" +
                      str(msg.command)+") received. Just for the record")
        self.commands.append(msg)

    def trajectory_callback(self, msg):
        rospy.loginfo("Node [" + rospy.get_name() + "] trajectory received")
        self.trajectories = msg
        self.curr_chunk = 0

    def sendNext(self):
        '''
        Let's divide 1 trajectory with N chunks into N trajectories with 1 chunk
        '''
        ans = False
        # do we have anything to transmit?
        if (self.curr_chunk >= 0):
            # do we have any chunk to transmit?
            if self.curr_chunk < len(self.trajectories.chunks):
                # index to my chunk list
                chunk = self.trajectories.chunks[self.curr_chunk]
                self.curr_chunk = self.curr_chunk + 1

                chunk.sequence_num = 0
                chunk.traj_id = self.local_traj_id
                self.local_traj_id = self.local_traj_id + 1
                chunk.final = True

                # constraints
                chunk.constraints = self.getCurrentConstraints(chunk.constraints)

                trajCV = ControllerTrajectoryChunkVec()
                trajCV.chunks = []
                trajCV.chunks.append(chunk)
                self.transmitCommands(chunk.traj_id)
                self.retransmitTraj(trajCV)
                ans = True
            # finished transmitting...
            else:
                self.curr_chunk = -1
        # tells if has transmitted
        return ans

    def transmitCommands(self, traj_id):
        comm = ControllerCommand()
        comm.robot_id = self.robot_id
        comm.command = comm.COMMAND_ACTIVATE
        comm.traj_id = traj_id
        self.command_pub.publish(comm)
        rospy.loginfo("Node [" + rospy.get_name() + "] activate sent")

        comm.command = comm.COMMAND_STARTTIME
        comm.start_time = rospy.Time.now()
        self.command_pub.publish(comm)
        rospy.loginfo("Node [" + rospy.get_name() + "] start time sent")

    def retransmitTraj(self, chunkVec):
        self.trajectory_pub.publish(chunkVec)

        ids = []
        for chunk in chunkVec.chunks:
            ids.append(str(chunk.traj_id)+', '+str(chunk.sequence_num))
        ids = ','.join(ids)
        rospy.loginfo("Node [" + rospy.get_name() + "] retransmitted (traj,seq):"+ids)

    def getCurrentConstraints(self, myConstraints):

        # All three arrays must be of the same length.
        # Arrays can be empty.
        '''
        possible spatial constraints:
        a0   a1     b
        2x   +y <  14
        -x   +y <   3
        -x   -y <   5
        3x -20y <  -1
        '''
        # myConstraints.spatial_coef_a0 = [2,  -1, -1,   3]
        # myConstraints.spatial_coef_a1 = [1,   1, -1, -20]
        # myConstraints.spatial_coef_b = [14,   3,  5,  -1]

        if hasattr(self, 'spatial_coef_a0'):
            myConstraints.spatial_coef_a0 = self.spatial_coef_a0
            myConstraints.spatial_coef_a1 = self.spatial_coef_a1
            myConstraints.spatial_coef_b = self.spatial_coef_b

        # Two numbers: minimal and maximal bounds // or an empty array]
        # myConstraints.bounds_tangential_velocity =[ config["bounds_tangential_velocity_min"], config["bounds_tangential_velocity_max"]]
        if hasattr(self, 'bounds_tangential_velocity'):
            myConstraints.bounds_tangential_velocity = self.bounds_tangential_velocity

        # Two numbers: minimal and maximal bounds // or an empty array]
        # myConstraints.bounds_steering_velocity =[ config["bounds_steering_velocity_min"], config["bounds_steering_velocity_max"]]
        if hasattr(self, 'bounds_steering_velocity'):
            myConstraints.bounds_steering_velocity = self.bounds_steering_velocity

        # TODO other potential constraints...
        # Two numbers: minimal and maximal bounds // or an empty array]
        # myConstraints.bounds_orientation =[ config["bounds_orientation_min"], config["bounds_orientation_max"]]

        # Two numbers: minimal and maximal bounds // or an empty array]
        # myConstraints.bounds_tangential_acceleration =[ config["bounds_tangential_acceleration_min"], config["bounds_tangential_acceleration_max"]]

        # One number or an empty array
        # myConstraints.bounds_centripetal_acceleration = config["bounds_centripetal_acceleration_min"]

        return myConstraints

    # def modulateStep(self, traject_ChV):
    #     '''
    #     takes a full chunk vector and mingles with its positions/speeds
    #     controller accepts it
    #     '''
    #     for chunk in traject_ChV.chunks:
    #         for step in chunk.steps:
    #             step.velocities.tangential = 2.0 * step.velocities.tangential
    #             step.velocities.steering = 2.0 * step.velocities.steering
    #             step.state.position_x = 0.5 + step.state.position_x
    #             step.state.position_y = 0.5 + step.state.position_y
    #
    #     self.retransmit(self.trajectories)


# Main function.
if __name__ == '__main__':
    rospy.init_node('DynamicConstraints')  # , log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = DynamicConstraints()
    except rospy.ROSInterruptException:
        pass
