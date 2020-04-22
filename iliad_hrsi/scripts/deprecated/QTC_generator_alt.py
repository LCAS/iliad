#!/usr/bin/env python

from geometry_msgs.msg import Vector3
'''

Calculates QTCc states from human and robot positions
Most of this node is ripped from QSR2constraints_node, keeping only state calculus.

It tries to guess future states from speeds 
'''
import uuid
import rospy
import numpy as np
from geometry_msgs.msg import Vector3Stamped, PoseStamped, PoseWithCovarianceStamped, TwistStamped, TwistWithCovarianceStamped, Point
from std_msgs.msg import Header, UInt64
from std_msgs.msg import String
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


class QTCStatePublisherNode():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params and atributes
        self.data_lock = Lock()

        self.closest_human_pose = None
        self.closest_human_twist = None

        # robot position                
        self.prev_robotPoseSt = None
        self.robotPoseSt = None

        # timestamp to get speeds...
        self.prev_time = rospy.Time.now()
        self.robotPose_time = rospy.Time.now()
        # qsrlib data...
        self.qsrlib = QSRlib()

        # TODO: add as parameter
        self.which_qsr = 'qtccs'
        
        # TODO: add as parameter
        self.dynammic_args = {"qtccs": {"no_collapse": True, "quantisation_factor": 0.01,
                                        "validate": False, "qsrs_for": [("human_os", "robot_os")]}}
        
        # sensible default values ...
        self.closest_human_pose = None
        self.closest_human_twist = None
        

        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [" + rospy.get_name() + "] entering spin...")


    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4)

        # human detection method: peopletracker(bayes, from STRANDS) OR trackedpersons (SPENCER)
        self.human_detection_method = rospy.get_param(
            '~human_detection_method', 'trackedpersons')

        # human detections using bayes peopletracker
        self.peopletracker_topic = rospy.get_param(
            '~peopletracker_topic', '/robot'+str(self.robot_id)+'/people_tracker/positions')

        # human tracking trackedpersons topic
        self.human_tracking_topic = rospy.get_param(
            '~human_tracking_topic', '/robot'+str(self.robot_id)+'/perception/tracked_persons')

        # publisher for our detection
        self.qtc_state_topic = rospy.get_param(
            '~qtc_state_topic', '/robot'+str(self.robot_id)+'/qsr/qtc_state')


        # points used in qtc is also published
        self.qtc_points_topic_name = rospy.get_param('~qtc_points_topic_name', '/robot' + str(self.robot_id) + '/qsr/qtc_points')

        # current robot position from tf tree
        self.robot_pose_topic_name = rospy.get_param('~robot_pose_topic_name', '/robot' + str(self.robot_id) + '/robot_poseST')

        # How far in the future we consider next state 
        self.sampling_time = rospy.get_param(
            '~sampling_time', 0.6)    

        # global frame id: both robot and human pose will be refered to this one
        self.global_frame_id = rospy.get_param('~global_frame', 'map_laser2d')
        # in tf2, frames do not have the initial slash
        if (self.global_frame_id[0] == '/'):
            self.global_frame_id = self.global_frame_id[1:]
        
        # tranform tf_timeout
        timeout = rospy.get_param('~tf_timeout', 2)
        self.tf_timeout = rospy.Duration(timeout)

    def initROS(self):
        # publishers
        self.qtc_state_pub = rospy.Publisher(self.qtc_state_topic, String, queue_size=1)
        self.qtc_points_pub = rospy.Publisher(self.qtc_points_topic_name, Float64MultiArray, queue_size=1)

        # service clients
        # none here

        # subscribers and listeners
        rospy.Subscriber(self.robot_pose_topic_name, PoseStamped, self.robot_pose_callback, queue_size=1)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # we either use spencer or bayes ...
        if self.human_detection_method=='peopletracker':
            rospy.Subscriber(self.peopletracker_topic, PeopleTracker, self.peopletracker_callback, queue_size=1)
        elif self.human_detection_method=='trackedpersons':
            rospy.Subscriber(self.human_tracking_topic, TrackedPersons, self.spencer_human_tracking_callback, queue_size=1)
        else:
            rospy.logerr("Node [" + rospy.get_name() + "] Provided detection method  ("+self.human_detection_method+") is not peopletracker/trackedpersons. Defaulting to trackedpersons")
            rospy.Subscriber(self.human_tracking_topic, TrackedPersons,
                        self.spencer_human_tracking_callback, queue_size=1)

        # service servers
        # none here

    # .............................................................................................................

    # we use robot pose topic to know robot position and speed
    def robot_pose_callback(self, msg):
        with self.data_lock:   
            self.prev_time = self.robotPose_time
            self.prev_robotPoseSt = self.robotPoseSt
            self.robotPose_time = rospy.Time.now()            
            self.robotPoseSt = self.transformPoseSt( msg, self.global_frame_id)        
            

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
                (dist, human_pose_glob) = self.getDistToHuman(human_pose)
                #rospy.loginfo("ID: "+str(i)+" Dist: "+str(dist) +" Pose:\n"+str(human_pose))
                if dist < min_dist:
                    min_i = i
                    min_dist = dist
                    self.closest_human_pose = human_pose_glob

                    self.closest_human_twist = self.transformTwistWC(trk_person.twist, msg.header, self.global_frame_id)
                    (xh0, yh0, hh0, vh0, wh0) = self.getHumanState()

                    rospy.logdebug_throttle(1, "Node [" + rospy.get_name() + "] " +
                                            "Closest human at: Pose ( " +
                                            str(xh0) + ", " + str(yh0) + ", " +
                                            str(hh0*180.0/np.pi) + " deg), " +
                                            "Speed ( " +
                                            str(vh0) + " m/sec, " +
                                            str(wh0*180.0/np.pi) + " deg/sec) "
                                            )
            #rospy.loginfo("...........................................................")
            if (min_dist>0):
                self.publishQTCdata()

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

    def transformTwistWC(self, twistWC, header, frame_id_out):
        """Returns a TwistWithCovarianceStamped in base frame"""
        # Create the stamped object
        twistS = TwistStamped()
        twistS.header = header
        twistS.twist = twistWC.twist

        twistS_base = self.transformTwist(frame_id_out, twistS)

        twistWC_out = TwistWithCovarianceStamped()
        twistWC_out.header = twistS_base.header
        twistWC_out.twist.twist = twistS_base.twist
        twistWC_out.twist.covariance = twistWC.covariance

        return twistWC_out

    def transformPoseSt(self, poseSt_in, frame_id_out):
        """Transform pose into provided frame id."""
        poseSt_out = None
        now = rospy.Time.now()

        # in tf2, frames do not have the initial slash
        if (frame_id_out[0] == '/'):
            frame_id_out = frame_id_out[1:]
        # in tf2, frames do not have the initial slash
        if (poseSt_in.header.frame_id[0] == '/'):
            poseSt_in.header.frame_id = poseSt_in.header.frame_id[1:]

        try:
            transform = self.tfBuffer.lookup_transform(frame_id_out, poseSt_in.header.frame_id, now, self.tf_timeout)
            poseSt_out = tf2_geometry_msgs.do_transform_pose(poseSt_in, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("[%s] transform from (%s) to (%s) failed: (%s).", rospy.get_name(), poseSt_in.header.frame_id, frame_id_out, e)        
        
        return poseSt_out

    def fromRobot2GlobalFrame(self, x, y, h):
        xg = yg = hg = None
        poseSt = PoseStamped()
        poseSt.header.frame_id = self.robotPoseSt.header.frame_id
        poseSt.pose.position.x = x
        poseSt.pose.position.y = y
        poseSt.pose.position.z = 0
        poseSt.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, h))

        if (self.robotPoseSt.header.frame_id!=self.global_frame_id):
            poseStG = self.transformPoseSt(poseSt, self.global_frame_id)
        else:
            poseStG = poseSt

        if (poseStG):
            xg = poseStG.pose.position.x
            yg = poseStG.pose.position.y
            hg = self.getRotation(poseStG.pose.orientation)
        
        return (xg,yg,hg)

    def getDistToHuman(self, human_pose_in):
        """Returns distance between human track and robot."""
        dist = np.inf        
        hpose_out = None

        if (self.robotPoseSt):
            hpose = PoseStamped()
            hpose.header = human_pose_in.header
            hpose.pose = human_pose_in.pose.pose

            human_pose_rob = self.transformPoseSt(hpose, self.robotPoseSt.header.frame_id)
            human_pose_glob = self.transformPoseSt(hpose, self.global_frame_id)

            if (human_pose_rob) and (human_pose_glob):
                dist = np.sqrt((human_pose_rob.pose.position.x ** 2) +
                            (human_pose_rob.pose.position.y ** 2))

                hpose_out = PoseWithCovarianceStamped()
                hpose_out.header = human_pose_glob.header
                hpose_out.pose.pose = human_pose_glob.pose
                hpose_out.pose.covariance = human_pose_in.pose.covariance

        return (dist, hpose_out)

    def getNextRobotState(self):
        xr1 = yr1 = hr1 = None
        inc_x = inc_y = inc_h = dt = 0
        if (self.prev_robotPoseSt and self.robotPoseSt and (self.robotPose_time > self.prev_time) ):
                t = [1, 2, 3]
                p = [3, 2, 0]
                np.interp(2.5, t, p)


                inc_x = self.robotPoseSt.pose.position.x - self.prev_robotPoseSt.pose.position.x
                inc_y = self.robotPoseSt.pose.position.y - self.prev_robotPoseSt.pose.position.y                
                inc_h = self.getRotation(self.robotPoseSt.pose.orientation) - self.getRotation(self.prev_robotPoseSt.pose.orientation)        
                dt = (self.sampling_time ) / (self.robotPose_time - self.prev_time).to_sec()

        if self.robotPoseSt:
            xr1 = self.robotPoseSt.pose.position.x + inc_x*dt
            yr1 = self.robotPoseSt.pose.position.y + inc_y*dt
            hr1 = self.getRotation(self.robotPoseSt.pose.orientation) + inc_h*dt

            rospy.logdebug_throttle(5, "Node [" + rospy.get_name() + "] " + "\n " + 
                                        "Robot status: Pose 0 ( " +
                                        '{0:.2f}'.format(self.prev_robotPoseSt.pose.position.x) + ", " + '{0:.2f}'.format(self.prev_robotPoseSt.pose.position.y) + ", " +
                                        '{0:.2f}'.format(self.getRotation(self.prev_robotPoseSt.pose.orientation) *180.0/np.pi) + " deg) " + 
                                        " at " + '{0:.2f}'.format(self.prev_time.to_sec() ) + " secs." + "\n" +                                     
                                        "Robot status: Pose 1 ( " +
                                        '{0:.2f}'.format(self.robotPoseSt.pose.position.x) + ", " + '{0:.2f}'.format(self.robotPoseSt.pose.position.y) + ", " +
                                        '{0:.2f}'.format(self.getRotation(self.robotPoseSt.pose.orientation) *180.0/np.pi) + " deg) " + 
                                        " at " + '{0:.2f}'.format(self.robotPose_time.to_sec() ) + " secs." + "\n" + 
                                        "Robot status: Pose 2( " +
                                        '{0:.2f}'.format(xr1) + ", " + '{0:.2f}'.format(yr1) + ", " +
                                        '{0:.2f}'.format(hr1 *180.0/np.pi) + " deg) " +
                                        " at " + '{0:.2f}'.format(self.sampling_time+self.robotPose_time.to_sec() ) + " secs." + "\n"
                                        )

        return (xr1, yr1, hr1)


    def publishQTCdata(self):
            # this method is called by one of the other callbacks, so its under the lock already
            #with self.data_lock:
            validData = self.closest_human_pose and self.closest_human_twist and self.robotPoseSt
            if validData:                
                # all should be already in global frame ...                
                (xh0, yh0, hh0, vh0, wh0) = self.getHumanState()
                (xh1, yh1, hh1) = self.getNextHumanState(xh0, yh0, hh0, vh0, wh0)

                xr0 = self.robotPoseSt.pose.position.x
                yr0 = self.robotPoseSt.pose.position.y
                hr0 = self.getRotation(self.robotPoseSt.pose.orientation)
                (xr1, yr1, hr1) = self.getNextRobotState()

                # get state only if transforms where successfull
                if ((xh0!=None) and (xh1!=None) and(xr0!=None) and(xr1!=None)):                                   
                    (isValid, state) = self.getQSRState(xh0, yh0, xh1, yh1, xr0, yr0, xr1, yr1)
                    self.qtc_state = state
                    self.is_valid = isValid

                    # Finally publish .......................                    
                    if self.is_valid:
                        self.qtc_state_pub.publish(self.qtc_state)
                        qtc_data = Float64MultiArray()
                        qtc_data.data = [xh0, yh0, hh0, xh1, yh1, hh1, xr0, yr0, hr0, xr1, yr1, hr1]
                        self.qtc_points_pub.publish(qtc_data)

            rospy.logdebug_throttle(1, "Node [" + rospy.get_name() + "] " +
                                                "Updated visuals ... "
                                                )

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


# Main function.
if __name__ == '__main__':
    rospy.init_node('QTCStatePublisherNode') # log_level=rospy.DEBUG)
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        goGo = QTCStatePublisherNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
