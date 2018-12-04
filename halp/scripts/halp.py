#!/usr/bin/env python

"""
Connects to trajectory topic (from v.e.n.) and report (from mpc)
Creates a list of goals for dwa to track
(geometry_msgs::PoseStamped)
"""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from orunav_msgs.msg import ControllerTrajectoryChunkVec, ControllerReport, \
    ControllerCommand, ControllerTrajectoryChunk, \
    ControllerTrajectoryStep


from nav_msgs.srv import GetPlan
from nav_msgs.msg import Odometry

import tf
import tf2_ros
import tf2_geometry_msgs

from threading import Lock
import fractions
import trajectoryPlanner

# from actionlib_msgs.msg import GoalStatusArray, GoalStatus
# from  functools import cmp_to_key


class Halp():
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Class attributes:
        self.path_mutex = Lock()
        # dictionary indexed by global trajectories id, containing paths with
        # all steps from chunks in given trajectory
        # why storing paths? because it's easy to display in rviz
        self.paths = dict()

        # mapping between global (traj, chunk,step) and local (traj,chunk,step)
        self.global_to_local_map = dict()
        # reverse of the above, stored to save time
        self.local_to_global_map = dict()

        # current trajectory id, from the ven point of view
        self.global_traj_id = -1
        # current trajectory id, from the mpc point of view
        self.local_traj_id = -1

        # initial poses for trajectory planning
        self.robot_pose = PoseStamped()
        self.robot_steering = 0.0

        # mpc report status
        self.status = -1
        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("[" + rospy.get_name() + "] " + "Node STARTED")

        rospy.spin()

    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)
        self.goal_frame_id = rospy.get_param('~goal_frame_id', 'world')

        # how many steps ahead we use for target
        self.look_ahead = rospy.get_param('~look_ahead', 80)

        # how many steps per chunk
        self.chunk_steps = rospy.get_param('~chunk_steps', 10)

        self.move_base_global_frame_id = rospy.get_param(
            '~move_base_global_frame_id', '/map_laser2d')

        # Used for kinematics
        # time between steps
        self.time_step = rospy.get_param('~time_step', 0.06)
        # distance between turning wheel and fixed ones
        self.wheel_base = rospy.get_param('~wheel_base', 1.19)

        # for debugging
        defaultVal = '/robot' + str(self.robot_id) + \
            '/control/controller/nextGoal'
        self.goals_topic = rospy.get_param('~goals_topic', defaultVal)

        # for debugging
        defaultVal = '/robot' + str(self.robot_id) + '/control/controller/path'
        self.path_topic = rospy.get_param('~path_topic', defaultVal)

        defaultVal = '/robot' + str(self.robot_id) + \
            '/control/controller/commands'
        self.in_command_topic = rospy.get_param(
            '~in_command_topic', defaultVal)
        self.out_command_topic = rospy.get_param(
            '~out_command_topic', '/robot' + str(self.robot_id) + '/control/controller/commands_mpc')

        self.in_reports_topic = rospy.get_param(
            '~in_reports_topic', '/robot' + str(self.robot_id) + '/control/controller/reports_mpc')
        self.out_reports_topic = rospy.get_param(
            '~out_reports_topic', '/robot' + str(self.robot_id) + '/control/controller/reports')

        self.in_trajectory_topic = rospy.get_param(
            '~in_trajectory_topic', '/robot' + str(self.robot_id) + '/control/controller/trajectories')
        self.out_trajectory_topic = rospy.get_param(
            '~out_trajectory_topic', '/robot' + str(
                self.robot_id) + '/control/controller/trajectories_mpc')

        self.planner_service_name = rospy.get_param(
            '~planner_srv_name', '/robot' + str(self.robot_id) + '/move_base/make_plan')

        # also posestamped should work. Anything that get us robot pose
        self.odom_topic = rospy.get_param(
            '~odom_topic', '/robot' + str(self.robot_id) + '/odom')

    def initROS(self):
        # only for debug purposes
        self.goal_pub = rospy.Publisher(
            self.goals_topic, PoseStamped, queue_size=1)
        # only for debug purposes
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1)

        # Needed to cast poses
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.trajectory_pub = rospy.Publisher(
            self.out_trajectory_topic,
            ControllerTrajectoryChunkVec,
            queue_size=1)
        self.out_command_pub = rospy.Publisher(
            self.out_command_topic, ControllerCommand, queue_size=1)
        self.out_reports_pub = rospy.Publisher(
            self.out_reports_topic, ControllerReport, queue_size=1)

        # move_base getPlan server client
        rospy.loginfo("[" + rospy.get_name() + "] " +
                      "Waiting for move_base GetPlan service (" + self.planner_service_name + ")")
        rospy.wait_for_service(self.planner_service_name)
        rospy.loginfo("[" + rospy.get_name() + "] " + "Available. Connecting...")
        self.getPlanCli = rospy.ServiceProxy(
            self.planner_service_name, GetPlan)
        rospy.loginfo("[" + rospy.get_name() + "] " + "Connected to move_base GetPlan service")

        # subscribe to robot odometry to get start poses in plans
        self.robot_odom_subs = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback)

        # Leave subscribers for the end to prevent errors
        rospy.Subscriber(
            self.in_trajectory_topic,
            ControllerTrajectoryChunkVec,
            self.trajectory_callback,
            queue_size=1)
        rospy.Subscriber(self.in_command_topic, ControllerCommand,
                         self.command_callback, queue_size=1)
        rospy.Subscriber(self.in_reports_topic, ControllerReport,
                         self.reports_callback, queue_size=1)
        rospy.loginfo("[" + rospy.get_name() + "]" +
                      " ROS publisher,services and subscribers established")

    def trajectory_callback(self, msg):
        """ Processes an incoming v.e.n. trajectory (global)

            - builds a Path from trajectory.
            - builds a mapping between (chunk,step) of input trajectory and path poses
            - publishes trajectory as a path
        """
        rospy.loginfo("[" + rospy.get_name() + "]" + " [from V.E.N.] Trajectory received")
        rospy.loginfo("[" + rospy.get_name() + "]" + "Global Trajectory has " + str(len(msg.chunks)
                                                                                    ) + " chunks and " + str(len(msg.chunks[0].steps)) + " steps per chunk")

        self.path_mutex.acquire()
        path = Path()
        path.header.frame_id = self.goal_frame_id
        self.global_to_local_map = dict()

        curr_local_traj = 0
        curr_local_step = 0
        curr_local_chunk = 0
        used_steps = 0

        for chunk_ind in range(len(msg.chunks)):
            chunk = msg.chunks[chunk_ind]

            for goal_step_ind in range(len(chunk.steps)):
                goal_step = chunk.steps[goal_step_ind]

                aPoseSt = PoseStamped()
                aPoseSt.header.frame_id = self.goal_frame_id
                aPoseSt.pose.position.x = goal_step.state.position_x
                aPoseSt.pose.position.y = goal_step.state.position_y

                quat = tf.transformations.quaternion_from_euler(
                    0, 0, goal_step.state.orientation_angle)
                aPoseSt.pose.orientation.x = quat[0]
                aPoseSt.pose.orientation.y = quat[1]
                aPoseSt.pose.orientation.z = quat[2]
                aPoseSt.pose.orientation.w = quat[3]

                path.poses.append(aPoseSt)
                global_tuple = (msg.chunks[0].traj_id,
                                chunk_ind, goal_step_ind)

                '''
                    Global to local trajectory mapping:
                        - 1 to 1 step mapping: we don't create new steps... (may be problematic)
                        - local trajectories have look_ahead steps, divided in chunks of size "chunk_steps"
                        - 1 global trajectory of N chunks would have [10*N] steps turns into L local trajectories with M chunks and [look_ahead] steps in total
                '''

                if used_steps == self.look_ahead:
                    # current local trajectory already has max num of steps
                    used_steps = 0
                    curr_local_traj = curr_local_traj + 1
                    curr_local_chunk = 0
                    curr_local_step = 0
                if curr_local_step == self.chunk_steps:
                    used_steps = used_steps + 1
                    curr_local_traj = curr_local_traj
                    # current chunk is full
                    curr_local_chunk = curr_local_chunk + 1
                    curr_local_step = 0
                else:
                    used_steps = used_steps + 1
                    curr_local_traj = curr_local_traj
                    curr_local_chunk = curr_local_chunk
                    # default case
                    curr_local_step = curr_local_step + 1
                local_tuple = (curr_local_traj,
                               curr_local_chunk, curr_local_step)

                self.global_to_local_map[global_tuple] = local_tuple
                self.local_to_global_map[local_tuple] = global_tuple

        self.paths[msg.chunks[0].traj_id] = path
        # debug...
        self.path_pub.publish(path)
        self.path_mutex.release()
        self.store_trajectory_msg(msg, "veh_traj")

        # if if this trajectory was already commanded, this should start
        # transmitting it.
        self.commandNextLocalTrajectory()

    def store_trajectory_msg(self, traj_msg, source):
        # OUT =      X           Y           theta       phi         v             w
        # out_ext = OUT traj_id seq_num final constrains

        for chunk_ind in range(len(traj_msg.chunks)):
            chunk = traj_msg.chunks[chunk_ind]

            chunk.traj_id
            chunk.sequence_num
            chunk.final
            chunk.constraints

            for step_ind in range(len(chunk.steps)):
                step = chunk.steps[step_ind]

                step.state.position_x
                step.state.position_y
                step.state.orientation_angle
                step.state.steering_angle
                step.velocities.tangential
                step.velocities.steering

        pass

    def command_callback(self, msg):
        """
            Process a command from v.e.n. to orunav_mpc

            Possible commands:
                COMMAND_BRAKE     => forward
                COMMAND_ACTIVATE  => New global trajectory confirmed. Get ready for receiving a new global trajectory.
                COMMAND_STARTTIME => forward
                COMMAND_RECOVER   => forward
        """
        rospy.loginfo("[" + rospy.get_name() + "]" +
                      " [from V.E.N.] Received command: " + self.printCommand(msg.command))
        if (msg.command == msg.COMMAND_ACTIVATE):
            if self.robot_id == msg.robot_id:
                # we save global trajectory id we are suppposed to follow
                self.global_traj_id = msg.traj_id
                # reset selected goal
                self.curr_goal_ind = 0
        else:
            rospy.loginfo("[" + rospy.get_name() + "]" +
                          " [to M.P.C.] Sending command: " + self.printCommand(msg.command))
            self.out_command_pub.publish(msg)

    def reports_callback(self, msg):
        """
            Process reports from orunav_mpc detailing how is he fulfilling the local trajectory.

            ControllerReport.CONTROLLER_STATUS_WAIT:     finished my trajectory
            ControllerReport.CONTROLLER_STATUS_ACTIVE:   executing chunk
            ControllerReport.CONTROLLER_STATUS_FINALIZE: in last chunck?
            ControllerReport.CONTROLLER_STATUS_FAIL:     something whent wrong
            ControllerReport.CONTROLLER_STATUS_TERMINATE: ?

            MPC updates its position and reports locally executed (chunk,step).
            We "transform" that into the equivalent global (chunk,step) for ven,
            by publishing a report changing step number.

            After that, we continue publishing Trajectories.
        """

        self.printIfNew(msg.status, "from M.P.C.")

        # used for trajectories
        self.robot_steering = msg.state.steering_angle

        if (msg.status == ControllerReport.CONTROLLER_STATUS_WAIT):
            # we are idle...
            self.printIfNew(msg.status, "to V.E.N")
            self.out_reports_pub.publish(msg)
        elif (msg.status == ControllerReport.CONTROLLER_STATUS_ACTIVE):
            # change chunk,step from report msg...
            self.translateChunks(msg)
            # report we are active, but using global trajectory.
            self.printIfNew(msg.status, "to V.E.N")
            self.out_reports_pub.publish(msg)
        elif (msg.status == ControllerReport.CONTROLLER_STATUS_FINALIZE):
            if not self.moreToTransmit():
                self.translateChunks(msg)
                self.printIfNew(msg.status, "to V.E.N")
                self.out_reports_pub.publish(msg)
            else:
                # if we have more, it will be transmitted
                self.commandNextLocalTrajectory()
        elif (msg.status == ControllerReport.CONTROLLER_STATUS_FAIL):
            # report error to system
            self.printIfNew(msg.status, "to V.E.N")
            self.out_reports_pub.publish(msg)
        elif (msg.status == ControllerReport.CONTROLLER_STATUS_TERMINATE):
            # TODO not sure of what's this...
            self.printIfNew(msg.status, "to V.E.N")
            self.out_reports_pub.publish(msg)

        self.status = msg.status

    def printIfNew(self, status, source):
        if status != self.status:
            rospy.loginfo("[" + rospy.get_name() + "]" + " [" + source +
                          "] new report status: " + self.printStatus(status))

    def translateChunks(self, msg):
        local_chunk_ind = msg.traj_chunk_sequence_num
        local_step_ind = msg.traj_step_sequence_num
        (global_traj_id, global_chunk_ind, global_step_ind) = self.local_to_global_map[
            (self.local_traj_id, local_chunk_ind, local_step_ind)]
        msg.traj_chunk_sequence_num = global_chunk_ind
        msg.traj_step_sequence_num = global_step_ind

        rospy.loginfo("I just reached:")
        rospy.loginfo("    local (chunk,step)==(" + str(local_chunk_ind) +
                      ",  " + str(local_step_ind) + ")")
        rospy.loginfo("    global (chunk,step)==(" + str(global_chunk_ind) +
                      ",  " + str(global_step_ind) + ")")

    def moreToTransmit(self):
        """
            Returns true if there are more goals from global trajectory to turn into local trajectories
        """
        ans = False
        try:
            ans = (self.curr_goal_ind +
                   self.look_ahead) < len(self.paths[self.global_traj_id].poses)
        except BaseException:
            pass
        return ans

    def buildTraj(self, local_plan, traj_id, num_steps):
        """
            Build new oru_trajectory based on provided path.
        """
        startYaw = self.getYaw(self.robot_pose)

        (x, y, th, phi, v, w) = self.solveSystem(local_plan.poses,
                                                 self.robot_pose.pose.position.x,
                                                 self.robot_pose.pose.position.y,
                                                 startYaw)


        !!!! CHECK BUILT TRAJECTORY

        trajChV = ControllerTrajectoryChunkVec()

        chunk = ControllerTrajectoryChunk()
        chunk.robot_id = self.robot_id
        chunk.traj_id = traj_id
        chunk.sequence_num = 0
        chunk.final = False

        rospy.loginfo("[" + rospy.get_name() + "]" + " Building local trajectory")
        # divide poses in chunks of num_steps
        for i in range(0, len(x)):

            # If current trajectory chunk is full of steps, append to
            # trajectory and get a new one
            if len(chunk.steps) >= num_steps:
                oldChunk = chunk
                trajChV.chunks.append(oldChunk)
                chunk = ControllerTrajectoryChunk()
                chunk.robot_id = self.robot_id
                chunk.traj_id = traj_id
                chunk.sequence_num = oldChunk.sequence_num + 1
                chunk.final = False

            # build step
            step = ControllerTrajectoryStep()
            step.state.position_x = x[i]
            step.state.position_y = y[i]
            step.state.orientation_angle = th[i]
            step.state.steering_angle = phi[i]
            step.velocities.tangential = v[i]
            step.velocities.steering = w[i]

            # TODO: this one or the other?
            step.mode = step.MODE_1

            chunk.steps.append(step)

        # Append last chunk
        chunk.final = True
        trajChV.chunks.append(oldChunk)

        return trajChV

    def sendActivateCommand(self, oru_tcv):
        """
            Sends a command activating provided trajectory
        """
        command = ControllerCommand()
        command.robot_id = oru_tcv.chunks[0].robot_id
        command.command = command.COMMAND_ACTIVATE

        command.traj_id = oru_tcv.chunks[0].traj_id

        rospy.loginfo("[" + rospy.get_name() +
                      " [to M.P.C.] Sending ACTIVATE command with local traj #(" + str(command.traj_id) + ")")
        self.out_command_pub.publish(command)

    def commandNextLocalTrajectory(self):
        """
            Triggers execution of a new local command/trajectory.

            - Retrieve (look_ahead steps ahed) goal from our stored path
            - Ask service getPlan to build a path
            - Build new oru_trajectory based on that path
            - Send command activating this trajectory
            - Send trajectory itself.
        """

        if self.moreToTransmit():
            goal = self.setNextGoal()
            local_plan = self.callGetPlan(goal)

            self.local_traj_id = self.local_traj_id + 1
            oru_t = self.buildTraj(local_plan, self.local_traj_id, self.chunk_steps)
            self.sendActivateCommand(oru_t)
            rospy.loginfo("[" + rospy.get_name() + "]" +
                          " [to M.P.C.] new local trajectory #(" + str(self.local_traj_id) + ")")
            self.trajectory_pub.publish(oru_t)

    def setNextGoal(self):
        """
        Sets new goal in current global path.

        - Retrieve (look_ahead steps ahed) goal from our active stored path list
        - Sets current goal
        """
        self.path_mutex.acquire()
        ans = None
        next_goal = self.curr_goal_ind + self.look_ahead
        if next_goal >= len(self.paths[self.global_traj_id].poses):
            next_goal = len(self.paths[self.global_traj_id].poses) - 1

        if not (self.curr_goal_ind == next_goal):
            ans = self.paths[self.global_traj_id].poses[next_goal]
            self.curr_goal_ind = next_goal
        else:
            pass

        self.path_mutex.release()
        return ans

    def callGetPlan(self, goalP0):
        """
            Ask service getPlan to build a path
        """
        plan_i = None
        gotAPlan = False
        attempts = 0
        planTolerance = 0.1
        goalP = self.castPoseIntoFrame(goalP0, self.move_base_global_frame_id)
        startP = self.castPoseIntoFrame(
            self.robot_pose, self.move_base_global_frame_id)

        while not gotAPlan:
            try:
                plan_i = self.getPlanCli(
                    start=startP, goal=goalP, tolerance=planTolerance)
#                rospy.loginfo_throttle(1,"["+rospy.get_name()+"] GetPlan service call succeeded, got a plan with ("+str(len(plan_i.plan.poses))+") points")
#                rospy.loginfo_throttle(1,"["+rospy.get_name()+"] According to look_ahead we should have ("+str(self.look_ahead)+") points")
                gotAPlan = len(plan_i.plan.poses) > 0

            except rospy.service.ServiceException as e:
                if attempts < 10:
                    attempts = attempts + 1
                    planTolerance = planTolerance + 0.01
                    rospy.logwarn_throttle(
                        1,
                        "[" +
                        rospy.get_name() +
                        "] GetPlan service call failed. Relaxing tolerance and retry after 100ms")
                    rospy.sleep(0.1)
                else:
                    rospy.logerr_throttle(
                        1, "[" + rospy.get_name() + "] GetPlan Failed, this is not working. Abort")
                    gotAPlan = True

        return plan_i.plan

    def odom_callback(self, msg):
        self.robot_pose = PoseStamped()
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose

    ###################    Accessory functions ###################

    def printStatus(self, statusInt):
        switcher = {
            ControllerReport.CONTROLLER_STATUS_WAIT: "WAIT",
            ControllerReport.CONTROLLER_STATUS_FAIL: "FAIL",
            ControllerReport.CONTROLLER_STATUS_ACTIVE: "ACTIVE",
            ControllerReport.CONTROLLER_STATUS_FINALIZE: "FINALIZE",
            ControllerReport.CONTROLLER_STATUS_TERMINATE: "TERMINATE"
        }
        return switcher.get(statusInt, "Unknown state(" + str(statusInt) + ")")

    def printCommand(self, commandInt):
        switcher = {
            ControllerCommand.COMMAND_BRAKE: "BRAKE",
            ControllerCommand.COMMAND_ACTIVATE: "ACTIVATE",
            ControllerCommand.COMMAND_STARTTIME: "STARTTIME",
            ControllerCommand.COMMAND_RECOVER: "RECOVER",
        }
        return switcher.get(
            commandInt,
            "Unknown command (" + str(commandInt) + ")")

    def getRPY(self, pSt):
        angles = (np.nan, np.nan, np.nan)
        if isinstance(pSt, PoseStamped):
            # get pose yaw
            q = (pSt.pose.orientation.x,
                 pSt.pose.orientation.y,
                 pSt.pose.orientation.z,
                 pSt.pose.orientation.w)

            angles = tf.transformations.euler_from_quaternion(q)
        else:
            rospy.logerr("[%s] input is not a poseStamped, got a %s",
                         rospy.get_name(), type(pSt))

        return angles

    def getYaw(self, pSt):
        # get pose yaw
        angles = self.getRPY(pSt)
        yaw = angles[2]
        return yaw

    def diffPoses(self, p_end, p_start):
        aPoseSt = PoseStamped()
        aPoseSt.header.frame_id = self.goal_frame_id

        aPoseSt.pose.position.x = p_end.pose.position.x - p_start.pose.position.x
        aPoseSt.pose.position.y = p_end.pose.position.y - p_start.pose.position.y
        aPoseSt.pose.position.z = p_end.pose.position.z - p_start.pose.position.z

        angles_start = self.getRPY(p_start)
        angles_end = self.getRPY(p_end)

        quat = tf.transformations.quaternion_from_euler(
            angles_end[0] - angles_start[0],
            angles_end[1] - angles_start[1],
            angles_end[2] - angles_start[2])

        aPoseSt.pose.orientation.x = quat[0]
        aPoseSt.pose.orientation.y = quat[1]
        aPoseSt.pose.orientation.z = quat[2]
        aPoseSt.pose.orientation.w = quat[3]

        return aPoseSt

    def distBetweenPoses(self, p_end, p_start):
        diff = self.diffPoses(p_end, p_start)
        d = np.linalg.norm(
            np.array([diff.pose.position.x, diff.pose.position.y, diff.pose.position.z]))
        return d

    def angleBetweenPoses(self, p_end, p_start):
        diff = self.diffPoses(p_end, p_start)
        ang = np.arctan2(diff.pose.position.y, diff.pose.position.x)
        return ang

# this returns a sorted list of dict keys ...
#  sorted(global_2_local.keys(), key=cmp_to_key(compare_tuples))
    def compare_tuples(t1, t2):
        # trajectory,chunk,step
        (t11, t12, t13) = t1
        (t21, t22, t23) = t2

        if t11 > t21:
            return 1
        elif t11 < t21:
            return -1
        elif t12 > t22:
            return 1
        elif t12 < t22:
            return -1
        elif t13 > t23:
            return 1
        elif t13 < t23:
            return -1
        return 0

    def castPoseIntoFrame(self, poseIn, frame_id):

        poseOut = poseIn
        if frame_id not in poseIn.header.frame_id:
            rospy.logdebug(
                "[" +
                rospy.get_name() +
                "] " +
                "Input pose in frame (" +
                poseIn.header.frame_id +
                ") will be casted into frame (" +
                frame_id +
                ")")
            try:
                queryTF = poseIn.header.frame_id
                # tf2 thingie: tfs dont start by /
                if queryTF[0] == '/':
                    queryTF = queryTF[1:]
                if frame_id[0] == '/':
                    frame_id = frame_id[1:]

                t = rospy.Time()
                trans = self.tfBuffer.lookup_transform(frame_id, queryTF, t)
                poseOut = tf2_geometry_msgs.do_transform_pose(poseIn, trans)

                rospy.logdebug("[%s] Requested pose casted", rospy.get_name())

            except tf2_ros.LookupException as e:
                rospy.logerr("[%s] Can't transform pose: %s",
                             rospy.get_name(), str(e))
                poseOut = None

        return poseOut

    def extractData(self, poses):
        x = []
        y = []
        th = []
        for poseSt in poses:
            x.append(poseSt.pose.position.x)
            y.append(poseSt.pose.position.y)
            th.append(self.getYaw(poseSt))
        x = np.array(x)
        y = np.array(y)
        th = np.array(th)
        return (x, y, th)

    def lcm(self, a, b):
        return a * b / fractions.gcd(a, b)

    def interpolate(self, data, numSamples):
        numData = len(data)
        if numData < numSamples:
            # interpolated data sampling points [0,numSamples)
            t = np.linspace(0, numSamples - 1, numSamples)

            # data points are "equally distributed" [strong asumption] over
            # that sampling time
            t0 = np.linspace(0, numSamples - 1, numData)

            # really simple interpolation.
            data_i = np.interp(t, t0, data)
        elif numData > numSamples:
            # interpolate to a mcm to be able to get a subset
            # interpolated data sampling points [0,mcm)
            mcm = self.lcm(numSamples, numData)
            t = np.linspace(0, mcm - 1, mcm)
            # data points are "equally distributed" [strong asumption] over
            # that sampling time
            t0 = np.linspace(0, mcm - 1, numData)
            # really simple interpolation.
            data_i = np.interp(t, t0, data)
            data_i = data_i[::mcm / numSamples]
        else:
            data_i = data
        return data_i

    def solveSystem(self, poses, xs, ys, ths):
        """

        :param poses: points local nav algorithm wants us to visit
        :param xs: starting point x coordinate
        :param ys: starting point y coordinate
        :param ths: starting point orientation coordinate
        :return: state and control vectors (x, y, th, phi, v, w)
        """

        s0 = np.array([[xs], [ys], [ths], [0]])  # initial state
        s1 = np.array([[2], [1], [0], [0]])  # final state



        num_points = len(poses)
        tspan = np.arange(0, self.time_step * num_points, self.time_step)

        kGains = np.array([[4], [4]]) # "Take back one kadam to honor the Hebrew God, whose Ark this is."

        (state, control) = trajectoryPlanner.plan_car_trajectory(self.wheel_base, s0, s1, kGains, tspan, self.time_step)
        x = state[0,:].flatten()
        y = state[1,:].flatten()
        th = state[2,:].flatten()
        phi = state[3,:].flatten()
        v = control[0,:].flatten()
        w = control[1,:].flatten()

        # print("State shape:"+str(state.shape))
        # print("Control shape:" + str(control.shape))
        # print("x shape:" + str(x.shape))
        # print("v shape:" + str(v.shape))
        return (x, y, th, phi, v, w)

    def solveSystemOld(self, poses, xs, ys, ths):
        '''
        This method is not used, but mostly does what build traj does
        We have (x,y,th) sampled every ti, and we need (phi,v,w)
        We know that:
        x'     = cos(theta) * v
        y'     = sin(theta) * v
        th'    = tan(phi)/L * v
        phi'   = w

        Use numpy gradient to obtain approximations to (x',y',th')
        and solve above equations.

        '''
        (x0, y0, th0) = self.extractData(poses)
        # When I add current position to states, fails miserably because we have odd turns. See reactive_nav_fail.bag
        #x0 = np.insert(x0, 0, xs)
        #y0 = np.insert(y0, 0, ys)
        #th0 = np.insert(th0, 0, ths)

        x = self.interpolate(x0, self.look_ahead)
        y = self.interpolate(y0, self.look_ahead)
        th = self.interpolate(th0, self.look_ahead)

        dx = np.gradient(x, self.time_step)
        dy = np.gradient(y, self.time_step)
        dth = np.gradient(th, self.time_step)

        '''
        Prevent dividing by 0:
        V has two equations:
        v = dy/np.sin(th)   and  v = dx/np.cos(th)
        If sin(th) == 0, then  cos(th) == 1, thus
            v = dx
        '''
        th1 = th
        th1[th == 0] = 1.0
        v = dy / np.sin(th1)
        v[th == 0] = dx[th == 0]

        phi = np.arctan2(dth * self.wheel_base, v)
        dphi = np.gradient(phi, self.time_step)
        w = dphi
        return (x, y, th, phi, v, w)


# Main function.
if __name__ == '__main__':
    rospy.init_node('halp_node')  # , log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = Halp()
    except rospy.ROSInterruptException:
        pass
