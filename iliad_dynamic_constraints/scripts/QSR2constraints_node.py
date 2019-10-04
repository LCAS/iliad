#!/usr/bin/env python

from geometry_msgs.msg import Vector3
'''

QSR2constraints



'''
import uuid
import rospy
import numpy as np
from geometry_msgs.msg import Vector3Stamped, PoseStamped, PoseWithCovarianceStamped, TwistStamped, TwistWithCovarianceStamped, Point
from std_msgs.msg import Header, UInt64
from spencer_tracking_msgs.msg import TrackedPersons, TrackedPerson
from bayes_people_tracker.msg import PeopleTracker
from tf.transformations import euler_from_quaternion, quaternion_from_euler, translation_matrix, quaternion_matrix
from orunav_msgs.msg import ControllerState, ControllerReport
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
import tf2_geometry_msgs
import tf2_ros
from threading import Lock

from visualization_msgs.msg import MarkerArray, Marker

from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib.qsrlib import QSR_QTC_BC_Simplified


class DynamicConstraintsNode():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params and atributes
        self.data_lock = Lock()

        # states from mpc node
        self.state = None
        self.prev_state = None
        # timestamp to get speeds...
        self.prev_time = None

        # qsrlib data...
        self.qsrlib = QSRlib()
        # TODO: add as parameter
        self.which_qsr = 'qtccs'
        # TODO: add as parameter
        self.dynammic_args = {"qtccs": {"no_collapse": True, "quantisation_factor": 0.01,
                                        "validate": False, "qsrs_for": [("human_os", "robot_os")]}}
        # TODO: add as parameter
        # these should be valid which_qsr states ...
        self.state_forbidden = ['-,-,-,-']

        # sensible default values ...
        self.closest_human_pose = None
        self.closest_human_twist = None
        self.constraint_v = 0
        self.constraint_w = 0
        self.robot_x = 0
        self.robot_y = 0
        self.robot_h = 0
        self.robot_v = 0
        self.robot_w = 0
        # TODO: add as parameter
        self.allowed_min_v = 0
        self.allowed_max_v = 0
        self.allowed_min_w = 0
        self.allowed_max_w = 0

        # What possible speeds we can achieve
        self.vr_space_inc = np.linspace(-self.max_vr, self.max_vr, self.num_speed_points)
        self.wr_space_inc = np.linspace(-self.max_wr, self.max_wr, self.num_speed_points)

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [" + rospy.get_name() + "] entering spin...")

        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.updateVisuals()
            self.updateConstraints()
            self.sendNewConstraints()
            r.sleep()

    def find_param(self, param_name, rid, default_val):
        ans = default_val
        allParams = rospy.get_param_names()
        for pi in allParams:
            if (pi[1:7] == ('robot'+str(rid))):
                if param_name in pi:
                    ans = rospy.get_param(pi, ans)
                    break

        return ans

    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)

        # where do we publish speed constraints: v and w
        self.velocity_constraints_topic = rospy.get_param(
            '~velocity_constraints_topic', '/robot'+str(self.robot_id)+'/velocity_constraints')

        # human detection method: peopletracker OR trackedpersons
        self.human_detection_method = rospy.get_param(
            '~human_detection_method', 'trackedpersons')

        # human tracking trackedpersons topic
        self.human_tracking_topic = rospy.get_param(
            '~human_tracking_topic', '/robot'+str(self.robot_id)+'/perception/tracked_persons')

        # human detections using bayes peopletracker
        self.peopletracker_topic = rospy.get_param(
            '~peopletracker_topic', '/robot'+str(self.robot_id)+'/people_tracker/positions')

        # MPC reports with current robot state
        self.velocity_constraints_topic = rospy.get_param(
            '~velocity_constraints_topic', '/robot'+str(self.robot_id)+'/velocity_constraints')

        # MARKER for visual data
        self.visual_topic = rospy.get_param(
            '~visual_topic', '/robot'+str(self.robot_id)+'/velocity_constraints_markers')

        self.reports_topic = rospy.get_param(
            '~reports_topic', '/robot' + str(self.robot_id) + '/control/controller/reports')

        # lookahead time for positions
        self.sampling_time = rospy.get_param(
            '~sampling_time', 0.6)    

        # robot frame id
        self.base_frame_id = rospy.get_param(
            '~base_frame', '/robot'+str(self.robot_id)+'/base_link')
        # in tf2, frames do not have the initial slash
        if (self.base_frame_id[0] == '/'):
            self.base_frame_id = self.base_frame_id[1:]

        # global frame id
        self.global_frame_id = rospy.get_param('~global_frame', 'map_laser2d')
        # in tf2, frames do not have the initial slash
        if (self.global_frame_id[0] == '/'):
            self.global_frame_id = self.global_frame_id[1:]
        
        # tranform tf_timeout
        timeout = rospy.get_param('~tf_timeout', 2)
        self.tf_timeout = rospy.Duration(timeout)

        # how many points we want to test
        # TODO: add as parameter
        self.num_speed_points = 20

        # maximum max_tangential_velocity m/s
        self.max_vr = self.find_param('max_tangential_velocity', self.robot_id, 1.0)

        # maximum steering wheel angle in rads
        self.phi_max = self.find_param('max_steer_angle', self.robot_id, 1.45)

        # steering wheel to back wheels distance m
        self.L = self.find_param('car_wheel_base', self.robot_id, 1.19)

        # w_max  = tan(phi_max) V_max / L
        self.max_wr = self.max_vr * np.tan(self.phi_max) / self.L

    def initROS(self):
        # publishers
        self.visual_pub = rospy.Publisher(self.visual_topic, MarkerArray, queue_size=1)
        self.velocity_constraints_pub = rospy.Publisher(self.velocity_constraints_topic, Float64MultiArray, queue_size=1)

        # service clients
        # none here

        # subscribers and listeners
        rospy.Subscriber(self.reports_topic, ControllerReport,
                         self.reports_callback, queue_size=1)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # we either use spencer or bayes ...
        if self.human_detection_method=='peopletracker':
            rospy.Subscriber(self.peopletracker_topic, PeopleTracker,
                         self.peopletracker_callback, queue_size=1)

        elif self.human_detection_method=='trackedpersons':
            rospy.Subscriber(self.human_tracking_topic, TrackedPersons,
                         self.spencer_human_tracking_callback, queue_size=1)
        else:
            rospy.logerr("Node [" + rospy.get_name() + "] Provided detection method  ("+self.human_detection_method+") is not peopletracker/trackedpersons. Defaulting to trackedpersons")
            rospy.Subscriber(self.human_tracking_topic, TrackedPersons,
                        self.spencer_human_tracking_callback, queue_size=1)

        # service servers
        # none here

    # .............................................................................................................

    # we use robot reports to know robot position and speed
    def reports_callback(self, msg):
        with self.data_lock:
            nowTime = rospy.Time.now()
            self.prev_state = self.state
            self.state = msg.state
            self.robot_x = msg.state.position_x
            self.robot_y = msg.state.position_y
            self.robot_h = msg.state.orientation_angle
            self.robot_v = 0
            self.robot_w = 0

            if (self.prev_state and self.prev_time):
                inc_t = (nowTime - self.prev_time).to_sec()
                inc_x = self.state.position_x - self.prev_state.position_x
                inc_y = self.state.position_y - self.prev_state.position_y
                inc_r = np.sqrt((inc_x ** 2) + (inc_y ** 2))
                self.robot_v = inc_r/inc_t
                inc_h = self.state.orientation_angle - self.prev_state.orientation_angle
                self.robot_w = inc_h/inc_t

                rospy.logdebug_throttle(5, "Node [" + rospy.get_name() + "] " +
                                        "Robot status: Pose ( " +
                                        str(self.state.position_x) + ", " + str(self.state.position_y) + ", " +
                                        str(self.state.orientation_angle*180.0/np.pi) + " deg), " +
                                        "Speed ( " +
                                        str(self.robot_v) + " m/sec, " +
                                        str(self.robot_w * 180.0/np.pi) + " deg/sec) "
                                        )
            self.prev_time = nowTime
            #self.updateVisuals()

    def spencer_human_tracking_callback(self, msg):
        with self.data_lock:
            # after each callback, reset closests

            self.closest_human_pose = None
            self.closest_human_twist = None
            min_dist = np.inf
            min_i = -1

            # TODO: Maybe it's better if we low pass filter these tracks to keep
            #       just humans that have been detected for a minimum period of time
            #rospy.loginfo("...........................................................")
            for i, trk_person in enumerate(msg.tracks):
                # Create the stamped object
                human_pose = PoseWithCovarianceStamped()
                # msg contains a header with the frame id for all poses
                human_pose.header = msg.header
                human_pose.pose = trk_person.pose

                # from the list of tracked persons, find the closest ...
                (dist, human_pose_base) = self.getDistToHuman(human_pose)
                #rospy.loginfo("ID: "+str(i)+" Dist: "+str(dist) +" Pose:\n"+str(human_pose))
                if dist < min_dist:
                    min_i = i
                    min_dist = dist
                    self.closest_human_pose = human_pose_base
                    self.closest_human_twist = self.getTwistInBaseFrame(trk_person.twist, msg.header)

                    (xh0, yh0, hh0, vh0, wh0) = self.getHumanState()

                    rospy.logdebug_throttle(5, "Node [" + rospy.get_name() + "] " +
                                            "Closest human at: Pose ( " +
                                            str(xh0) + ", " + str(yh0) + ", " +
                                            str(hh0*180.0/np.pi) + " deg), " +
                                            "Speed ( " +
                                            str(vh0) + " m/sec, " +
                                            str(wh0*180.0/np.pi) + " deg/sec) "
                                            )
            #rospy.loginfo("...........................................................")
            #self.updateVisuals()
            #self.updateConstraints()
            #self.sendNewConstraints()


    def peopletracker_callback(self, msg):
        spencer_msg = self.bayes2spencer(msg)
        self.spencer_human_tracking_callback(spencer_msg)

    #  There is some info here that may be not accurate
    def bayes2spencer(self,bayes_msg):
        spencer_msg=TrackedPersons()
        spencer_msg.header = bayes_msg.header

        for i, pose in enumerate(bayes_msg.poses):
            track=TrackedPerson()
            track.track_id = self.string2uint64(bayes_msg.uuids[i])
            # PoseWithCovariance
            track.pose.pose.position = pose.position
            track.pose.pose.orientation = pose.orientation
            # TwistWithCovariance
            track.twist.twist.linear = bayes_msg.velocities[i]
            # Data not in bayes. Not sure about these ...
            track.pose.covariance = (np.random.normal(0.3,0.1)* np.identity(6)).flatten().tolist()
            track.twist.covariance = (np.random.normal(0.3,0.1)* np.identity(6)).flatten().tolist()
            # we assume 0 here
            # track.twist.twist.angular
            track.is_occluded = False
            track.is_matched = False
            track.detection_id = self.string2uint64(bayes_msg.uuids[i])
            track.age = 0

            spencer_msg.tracks.append(track)


        return spencer_msg

    def string2uint64(self, in_string):        
        ans=UInt64(uuid.UUID(in_string).int)

        return ans

    # ..............................
    # based on http://docs.ros.org/hydro/api/tf/html/c++/transform__listener_8cpp_source.html
    #

    def transformTwist(self, target_frame, msg_in):  # msg_in is a TwistStamped

        # divide twist message into rotational and translational velocities
        v_rot = [msg_in.twist.angular.x, msg_in.twist.angular.y, msg_in.twist.angular.z]
        v_trans = [msg_in.twist.linear.x, msg_in.twist.linear.y, msg_in.twist.linear.z]

        # get the transform between target frame and twist frame
        try:
            frame_tf = self.tfBuffer.lookup_transform(
                target_frame, msg_in.header.frame_id,  msg_in.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Node [" + rospy.get_name() + "] Can't find transform. (" + str(e) + ") ")
            return None

        q_ = frame_tf.transform.rotation
        x_ = frame_tf.transform.translation

        # get the translation matrix from translation vector
        trans_vec = [x_.x, x_.y, x_.z]

        # get the rotation matrix from quaternion.
        rot_mat = quaternion_matrix([q_.x, q_.y, q_.z, q_.w])

        # rotate vector using
        out_rot = np.dot(rot_mat[:3, :3], v_rot)
        out_vel = np.dot(rot_mat[:3, :3], v_trans) + np.cross(trans_vec, out_rot)

        interframe_twist = TwistStamped()
        # we asumme frames are not moving relatively? not sure

        msg_out = TwistStamped()
        msg_out.header.stamp = msg_in.header.stamp
        msg_out.header.frame_id = target_frame

        #
        msg_out.twist.linear.x = out_vel[0] + interframe_twist.twist.linear.x
        msg_out.twist.linear.y = out_vel[1] + interframe_twist.twist.linear.y
        msg_out.twist.linear.z = out_vel[2] + interframe_twist.twist.linear.z

        # final angular speed is input angular speed, rotated plus interframe speed
        msg_out.twist.angular.x = out_rot[0] + interframe_twist.twist.angular.x
        msg_out.twist.angular.y = out_rot[1] + interframe_twist.twist.angular.y
        msg_out.twist.angular.z = out_rot[2] + interframe_twist.twist.angular.z

        return msg_out  # geometry_msgs::TwistStamped
        # ..............................

    def getTwistInBaseFrame(self, twistWC, header):
        """Returns a TwistWithCovarianceStamped in base frame"""
        # Create the stamped object
        twistS = TwistStamped()
        twistS.header = header
        twistS.twist = twistWC.twist

        twistS_base = self.transformTwist(self.base_frame_id, twistS)

        twistWC_out = TwistWithCovarianceStamped()
        twistWC_out.header = twistS_base.header
        twistWC_out.twist.twist = twistS_base.twist
        twistWC_out.twist.covariance = twistWC.covariance

        return twistWC_out


    def transformPoseSt(self, poseSt_in, frame_id_out):
        """Transform pose into provided frame id."""
        poseSt_out = PoseStamped()

        try:
            poseSt_out = self.tfBuffer.transform( poseSt_in, frame_id_out, self.tf_timeout)
        except tf2_ros.TransformException as e:
            poseSt_out.header.frame_id = 'NONE'
            rospy.logerr("Node [" + rospy.get_name() + "] Can't transform. (" + str(e) + ") ")

        return (poseSt_out)

    def fromRobot2GlobalFrame(self, x, y, h):
        xg = yg = hg = None
        poseSt = PoseStamped()
        poseSt.header.frame_id = self.base_frame_id
        poseSt.pose.position.x = x
        poseSt.pose.position.y = y
        poseSt.pose.position.z = 0
        poseSt.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, h))

        poseStG = self.transformPoseSt(poseSt, self.global_frame_id)

        if (poseStG.header.frame_id == self.global_frame_id):
            xg = poseStG.pose.position.x
            yg = poseStG.pose.position.y
            hg = self.getRotation(poseStG.pose.orientation)
        
        return (xg,yg,hg)




    def getDistToHuman(self, human_pose_in):
        """Returns distance between human track and robot."""
        human_pose_base = None
        dist = np.inf        
        hpose_out = None
        hpose = PoseStamped()
        hpose.header = human_pose_in.header
        hpose.pose = human_pose_in.pose.pose

        human_pose_base = self.transformPoseSt(hpose, self.base_frame_id)

        if (human_pose_base.header.frame_id == self.base_frame_id):
            dist = np.sqrt((human_pose_base.pose.position.x ** 2) +
                           (human_pose_base.pose.position.y ** 2))
            hpose_out = PoseWithCovarianceStamped()
            hpose_out.header = human_pose_base.header
            hpose_out.pose.pose = human_pose_base.pose
            hpose_out.pose.covariance = human_pose_in.pose.covariance

        return (dist, hpose_out)

    def updateVisuals(self):
        with self.data_lock:
            validData = isinstance(self.closest_human_pose, PoseWithCovarianceStamped)  and isinstance(self.closest_human_twist, TwistWithCovarianceStamped) 
            
            if validData:
                
                (xh0, yh0, hh0, vh0, wh0) = self.getHumanState()
                (xh1, yh1, hh1) = self.getNextHumanState(xh0, yh0, hh0, vh0, wh0)
                xr0 = yr0 = hr0 = 0
                (xr1, yr1, hr1) = self.getNextHumanState(xr0, yr0, hr0, self.robot_v, self.robot_w)

                tx = xh0/2.0
                ty = yh0/2.0

                # All that data is in robot frame, but we need it in global frame...
                (xh0, yh0, hh0) = self.fromRobot2GlobalFrame(xh0, yh0, hh0)
                (xh1, yh1, hh1) = self.fromRobot2GlobalFrame(xh1, yh1, hh1)
                # I could just use getOrigin with this one ...
                (xr0, yr0, hr0) = self.fromRobot2GlobalFrame(xr0, yr0, hr0)
                (xr1, yr1, hr1) = self.fromRobot2GlobalFrame(xr1, yr1, hr1)

                # plot only if transforms where successfull
                if ((xh0!=None) and (xh1!=None) and(xr0!=None) and(xr1!=None)):

                    data = MarkerArray()

                    # 0 line
                    line = Marker()
                    line.id = 0
                    line.type = Marker.LINE_STRIP
                    line.header.frame_id = self.global_frame_id
                    line.header.stamp = rospy.Time.now()
                    line.ns = "connecting_line"
                    line.action = Marker.ADD
                    line.pose.orientation.w = 1.0

                    # LINE_STRIP markers use only the x component of scale, for the line width
                    line.scale.x = 0.05

                    # blue color
                    line.color.b = 1.0
                    line.color.a = 1.0

                    humanP = Point(xh0, yh0, 1)
                    
                    robotP = Point(xr0, yr0, 1)
                    line.points.append(humanP)
                    line.points.append(robotP)

                    data.markers.append(line)

                    # 1 text
                    text = Marker()
                    text.id = 1
                    text.type = Marker.TEXT_VIEW_FACING
                    text.header.frame_id = self.base_frame_id
                    text.header.stamp = rospy.Time.now()
                    text.ns = "descriptor"
                    text.action = Marker.ADD
                    text.pose.orientation.w = 1.0

                    text.pose.position.x = tx
                    text.pose.position.y = ty
                    text.pose.position.z = 1+0.2
                    # TEXT_VIEW_FACING markers use only the z component of scale, specifies the height of an uppercase "A".
                    text.scale.z = 0.45

                    # we show next state as text
                    (isValid, state) = self.getQSRState(xh0, yh0, xh1, yh1, xr0, yr0, xr1, yr1)

                    if isValid:
                        text.text = 'QTC State: ' + state
                        # yellow color
                        text.color.r = text.color.g = 1.0
                        text.color.a = 1.0
                    else:
                        text.text = 'Unknown state'
                        # red color
                        text.color.r = 1.0
                        text.color.a = 1.0
                        
                    data.markers.append(text)

                    # Finally publish .......................
                    self.visual_pub.publish(data)

        rospy.logdebug_throttle(1, "Node [" + rospy.get_name() + "] " +
                                                "Updated visuals ... "
                                                )


    def isBigger(self, rel_amount, amount1, amount0):
        dif = np.abs(amount0-amount1)
        ans = True
        if amount0 > 0:
            rel_dif = dif/amount0
            ans = rel_dif > rel_amount
        # if original amount was 0, any value is infinite change ...
        else:
            ans = False
            
        return ans

    def getQSRState(self, xh0, yh0, xh1, yh1, xr0, yr0, xr1, yr1):
        td = self.sampling_time

        isValid = False
        state = None
        human_os = [Object_State(name="human_os", timestamp=0, x=xh0, y=yh0),
                    Object_State(name="human_os", timestamp=td, x=xh1, y=yh1)
                    ]

        robot_os = [Object_State(name="robot_os", timestamp=0, x=xr0, y=yr0),
                    Object_State(name="robot_os", timestamp=td, x=xr1, y=yr1)
                    ]

        # make some input data
        world = World_Trace()

        world.add_object_state_series(human_os)
        world.add_object_state_series(robot_os)

        # make a QSRlib request message
        qsrlib_request_message = QSRlib_Request_Message(
            self.which_qsr, world, self.dynammic_args)

        # request your QSRs
        qsrlib_response_message = self.qsrlib.request_qsrs(req_msg=qsrlib_request_message)

        # should have 1 timestamp
        t = qsrlib_response_message.qsrs.get_sorted_timestamps()
        if len(t) != 1:
            rospy.logerr("Node [" + rospy.get_name() +
                         "@getQSRState] : response timestamp message lenght is not 1.")
            return (isValid, state)
        t = t[0]

        # should have one value: human to robot
        v = qsrlib_response_message.qsrs.trace[t].qsrs.values()
        if len(v) != 1:
            rospy.logerr("Node [" + rospy.get_name() +
                         "@getQSRState] : response values message lenght is not 1.")
            return (isValid, state)
        v = v[0]
        state = v.qsr.values()
        if len(state) != 1:
            rospy.logerr("Node [" + rospy.get_name() +
                         "@getQSRState] : response state message lenght is not 1.")
            return (isValid, state)
        state = state[0]

        if state in self.state_forbidden:
            isValid = False
  	else:
        # Ok, maybe is valid ...
            isValid = True

        return (isValid, state)

    def getNextHumanState(self, xh0, yh0, hh0, vh0, wh0):
        """Simplest way to get next human position given current state """
        hh1 = hh0 + wh0*self.sampling_time
        xh1 = xh0 + vh0*np.cos(hh1) * self.sampling_time
        yh1 = yh0 + vh0*np.sin(hh1) * self.sampling_time
        return (xh1, yh1, hh1)

    def getHumanState(self):
        """Get human position, orientation and speeds from current tracking"""
        
        xh0 = self.closest_human_pose.pose.pose.position.x
        yh0 = self.closest_human_pose.pose.pose.position.y
        hh0 = self.getRotation(self.closest_human_pose.pose.pose.orientation)

        vh0 = np.sqrt((self.closest_human_twist.twist.twist.linear.x ** 2) +
                      (self.closest_human_twist.twist.twist.linear.y ** 2))
        wh0 = self.closest_human_twist.twist.twist.angular.z
        # np.atan2(self.closest_human_twist.linear.y,
        #          self.closest_human_twist.linear.x)
        return (xh0, yh0, hh0, vh0, wh0)

    def getRotation(self, orientation_q):
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        # quat = quaternion_from_euler(roll, pitch, yaw)
        return yaw

    def updateConstraints(self):
        with self.data_lock:
            # 1. get human pose and most likely speed
            if (self.closest_human_pose) and (self.closest_human_twist):
                # current human position
                (xh0, yh0, hh0, vh0, wh0) = self.getHumanState()

                # next likely human position
                (xh1, yh1, hh1) = self.getNextHumanState(xh0, yh0, hh0, vh0, wh0)
                rospy.loginfo_throttle(5, "Node [" + rospy.get_name() + "] " +
                                        "Next most likely human state: Pose ( " +
                                        str(xh1) + ", " + str(yh1) + ", " +
                                        str(hh1*180.0/np.pi) + " deg), " +
                                        "Speed ( " +
                                        str(vh0) + " m/sec, " +
                                        str(wh0*180.0/np.pi) + " deg/sec) "
                                        )

                # What possible speeds robot can achieve from current one
                vr_space = self.vr_space_inc + self.robot_v
                wr_space = self.wr_space_inc + self.robot_w

                # Create 2D speed space grid
                V, W = np.meshgrid(vr_space, wr_space)

                # robot position in base_link is ... 0!
                hr0 = 0
                xr0 = 0
                yr0 = 0
                # And from that, what posible points robot can reach
                Hr1 = hr0 + W*self.sampling_time
                Xr1 = xr0 + V*np.cos(Hr1) * self.sampling_time
                Yr1 = yr0 + V*np.sin(Hr1) * self.sampling_time

                T = np.zeros_like(Xr1).flatten()
                all_states = []
                for i in range(0, len(Xr1.flatten())):
                    xr1 = Xr1.flatten()[i]
                    yr1 = Yr1.flatten()[i]
                    hr1 = Hr1.flatten()[i]
                    (T[i], state) = self.isValidState(xh0, yh0,
                                                    xh1, yh1, xr0, yr0, xr1, yr1)

                    all_states.append(state)
                    # T[i]=1, allowed, =0 not allowed...

                T = T.reshape(Xr1.shape)
                max_area, min_i, min_j, max_i, max_j = self.maximalRectangle(T)
                self.allowed_min_v = V[min_i, min_j]
                self.allowed_max_v = V[max_i, max_j]
                self.allowed_min_w = W[min_i, min_j]
                self.allowed_max_w = W[max_i, max_j]

    def sendNewConstraints(self):
        with self.data_lock:
            newV = np.min([np.abs(self.allowed_min_v), np.abs(self.allowed_max_v)])
            newW = np.min([np.abs(self.allowed_min_w), np.abs(self.allowed_max_w)])

            if self.isBigger(0.05, newV, self.constraint_v) or self.isBigger(0.05, newW, self.constraint_w):
                self.constraint_v = newV
                self.constraint_w = newW
                msg = Float64MultiArray()
                msg.data.append(self.constraint_v)
                msg.data.append(self.constraint_w)
                self.velocity_constraints_pub.publish(msg)
                rospy.loginfo_throttle(3,"Node [" + rospy.get_name() + "] " +
                            "New speed Constraints sent : V ( " +
                            str(self.constraint_v) + " m/s), " +
                            "W ( " +
                            str(self.constraint_w * 180.0/np.pi) + " deg/sec) "
                            )


    def maximalRectangle(self, M):
        n, m = M.shape

        # Count number of 1 bits in each column
        cumul_height = np.zeros((n, m))

        # first row
        cumul_height[0, :] = M[0, :]

        # rest of matrix
        for i in range(1, n):
            for j in range(0, m):
                if M[i, j] == 1:
                    cumul_height[i, j] = cumul_height[i-1, j] + 1
                else:
                    cumul_height[i, j] = 0

        max_area = 0
        inc_i = 0
        inc_j = 0
        max_i = 0
        min_i = 0
        min_j = 0
        max_j = 0

        # for each row we
        for i in range(0, n):
            # We compute all contiguous sublists of each row
            row = cumul_height[i, :]
            for width in range(1, len(row) + 1):
                for offset in range(0, m - width + 1):
                    slice = row[0 + offset:offset + width]
                    slice_area = width * np.min(slice)
                    if slice_area > max_area:
                        max_area = slice_area
                        inc_i = int(np.min(slice))
                        inc_j = width  # len(slice)
                        max_i = i
                        min_i = max_i-inc_i+1
                        min_j = offset
                        max_j = min_j+inc_j-1

        return max_area, min_i, min_j, max_i, max_j

# Main function.
if __name__ == '__main__':
    rospy.init_node('DynamicConstraintsNode')#, log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = DynamicConstraintsNode()
    except rospy.ROSInterruptException:
        pass
