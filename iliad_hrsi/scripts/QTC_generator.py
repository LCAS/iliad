#!/usr/bin/env python

from geometry_msgs.msg import Vector3
'''

Calculates QTCc state from two latest human positions (separated at least tsample) and robot position. 
This one gets robot positions by checking the tf tree at the timestamp of the corresponding human position.

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
        self.closest_human_pose = None
        self.prev_closest_human_pose = None

        # qsrlib data...
        self.qsrlib = QSRlib()

        # TODO: add as parameter
        self.which_qsr = 'qtccs'
        
        # TODO: add as parameter
        self.dynammic_args = {"qtccs": {"no_collapse": True, "quantisation_factor": 0.01,
                                        "validate": False, "qsrs_for": [("human_os", "robot_os")]}}
        
        # sensible default values ...
        self.closest_human_pose = None
        

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

        # current robot frame ide from tf tree
        self.robot_base_frame_id = rospy.get_param('~robot_base_frame_id', '/robot' + str(self.robot_id) + '/base_link')
        # in tf2, frames do not have the initial slash
        if (self.robot_base_frame_id[0] == '/'):
            self.robot_base_frame_id = self.robot_base_frame_id[1:]

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

        # tf buffers
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # subscribers and listeners

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
                
    def spencer_human_tracking_callback(self, msg):
            min_dist = np.inf
            min_i = -1
            closest_human_pose = None
            # TODO: Maybe it's better if we low pass filter these tracks to keep
            #       just humans that have been detected for a minimum period of time
            #rospy.loginfo("...........................................................")
            for i, trk_person in enumerate(msg.tracks):
                # Create the stamped object
                human_pose = PoseStamped()
                # msg contains a header with the frame id for all poses
                human_pose.header = msg.header
                human_pose.pose = trk_person.pose.pose

                # from the list of tracked persons, find the closest to robot
                (dist, human_pose_glob) = self.getDistToHuman(human_pose)
                #rospy.loginfo("ID: "+str(i)+" Dist: "+str(dist) +" Pose:\n"+str(human_pose))
                if dist < min_dist:
                    min_i = i
                    min_dist = dist
                    closest_human_pose = human_pose_glob

            #rospy.loginfo("...........................................................")
            if (min_i>-1):
                # get first
                if (self.prev_closest_human_pose == None):
                    self.prev_closest_human_pose = closest_human_pose
                # get second and clear
                elif (self.closest_human_pose == None):
                    if (human_pose_glob.header.stamp-self.prev_closest_human_pose.header.stamp).to_sec()>self.sampling_time:
                        self.closest_human_pose = closest_human_pose
                        self.publishQTCdata()
                        self.prev_closest_human_pose = None
                        self.closest_human_pose = None

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

    def transformPoseSt(self, poseSt_in, frame_id_out, when):
        """Transform pose into provided frame id."""
        poseSt_out = None        

        # in tf2, frames do not have the initial slash
        if (frame_id_out[0] == '/'):
            frame_id_out = frame_id_out[1:]
        # in tf2, frames do not have the initial slash
        if (poseSt_in.header.frame_id[0] == '/'):
            poseSt_in.header.frame_id = poseSt_in.header.frame_id[1:]

        try:
            transform = self.tfBuffer.lookup_transform(frame_id_out, poseSt_in.header.frame_id, when, self.tf_timeout)
            poseSt_out = tf2_geometry_msgs.do_transform_pose(poseSt_in, transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("[%s] transform from (%s) to (%s) failed: (%s).", rospy.get_name(), poseSt_in.header.frame_id, frame_id_out, e)        
        
        return poseSt_out

    def getDistToHuman(self, human_pose_in):
        """Returns distance between human track and robot."""
        dist = np.inf        
        hpose_out = None

        now = rospy.Time.now()
        human_pose_rob = self.transformPoseSt(human_pose_in, self.robot_base_frame_id, now)
        human_pose_glob = self.transformPoseSt(human_pose_in, self.global_frame_id, now)

        if (human_pose_rob) and (human_pose_glob):
            dist = np.sqrt((human_pose_rob.pose.position.x ** 2) +
                        (human_pose_rob.pose.position.y ** 2))

            hpose_out = human_pose_glob

        return (dist, hpose_out)

    def getRobotState(self, timestamp):
        x = y = h = None

        robotPoseSt = PoseStamped()
        robotPoseSt.header.stamp = timestamp
        robotPoseSt.header.frame_id = self.robot_base_frame_id

        robotPoseGlob = self.transformPoseSt(robotPoseSt, self.global_frame_id, timestamp)
        if (robotPoseGlob):
            x = robotPoseGlob.pose.position.x
            y = robotPoseGlob.pose.position.y
            h = self.getRotation(robotPoseGlob.pose.orientation)
        return (x, y, h)

    def publishQTCdata(self):
            # this method is called by one of the other callbacks, so its under the lock already
            #with self.data_lock:
            validData = self.closest_human_pose and self.prev_closest_human_pose
            if validData:                
                # all should be already in global frame ...                
                (xh0, yh0, hh0) = self.getState(self.prev_closest_human_pose)
                (xh1, yh1, hh1) = self.getState(self.closest_human_pose)

                (xr0, yr0, hr0) = self.getRobotState(self.prev_closest_human_pose.header.stamp)
                (xr1, yr1, hr1) = self.getRobotState(self.closest_human_pose.header.stamp)

                # get state only if transforms where successfull
                if ((xh0!=None) and (xh1!=None) and(xr0!=None) and(xr1!=None)):                                   
                    (isValid, state) = self.getQSRState(xh0, yh0, xh1, yh1, xr0, yr0, xr1, yr1 , (self.closest_human_pose.header.stamp-self.prev_closest_human_pose.header.stamp).to_sec() )
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

    def getQSRState(self, xh0, yh0, xh1, yh1, xr0, yr0, xr1, yr1, td):
        
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

    def getState(self, poseSt):
        x = poseSt.pose.position.x
        y = poseSt.pose.position.y
        h = self.getRotation(poseSt.pose.orientation)
        return (x, y, h)

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
