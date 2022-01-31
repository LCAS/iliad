#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64MultiArray
from nav_msgs.msg import OccupancyGrid, Odometry
from map_msgs.msg import OccupancyGridUpdate
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int64, Float64, Empty
from spencer_tracking_msgs.msg import DetectedPersons, DetectedPerson, TrackedPersons, TrackedPerson
import math
import tf.transformations
from orunav_msgs.msg import Task, ComputeTaskStatus,ReplanStatus, ControllerCommand
import json


# topics to check
# robot1/velocity_constraints
# robot1/ebrake
# robor1/control/report
# mission_status
# coordinator/replan/status

# [robot_lin_x, robot_ang_z, closest_person_x, closest_person_y, closest_person_yaw, closest_person_lin_x, closest_person_lin_y, closest_human_distance,relative_speed_in_x]
# total_mission_time

num_breaks_2 = 0

class iliad_safey_metrics_saver(object):
    def __init__(self):

        self.test_name = rospy.get_param('~test_name', 'default_configuration')
        self.current_experiment = rospy.get_param('current_experiment', 0)

        #init variables
        self.active_robots = {}
        self.mission_status = {}
        self.recording = False
        self.num_hri_replans = 0
        self.num_hri_constraints = 0
        self.num_vsmu_constraints = 0
        self.num_brakes = 0
        self.starting_time = 0
        self.metrics_saved = False


        self.robot_lin_x = 0
        self.robot_ang_z = 0
        self.closest_person_x = 0
        self.closest_person_y= 0
        self.closest_person_yaw= 0
        self.closest_person_lin_x= 0
        self.closest_person_lin_y= 0
        self.closest_human_distance= 0
        self.relative_robot_human_speed_in_x= 0

        self.old_replan_status = -1
        self.old_controller_command = -1

        # Subscribers
        rospy.Subscriber("robot1/human_perception/tracked_persons_in_base_link", TrackedPersons, self.tracked_persons_callback, queue_size=1)
        rospy.Subscriber("robot1/odom", Odometry, self.robot_odometry_callback)
        rospy.Subscriber("active_robot_status",String, self.active_robot_status_callback,queue_size=1)
        rospy.Subscriber("mission_status", String, self.mission_status_callback,queue_size=1)
        rospy.Subscriber("robot1/control/controller/commands",ControllerCommand , self.controller_command_callback, queue_size=1)
        rospy.Subscriber("layer2_activated",Empty,self.layer2_activated_callback,queue_size=1)
        rospy.Subscriber("robot1/qsr/layer3_activated",Empty,self.layer3_activated_callback,queue_size=1)
        rospy.Subscriber("robot1/qsr/layer3_replan_triggered",Empty,self.layer3_replan_triggered_callback,queue_size=1)

        # Publishers
        self.recording_started_pub = rospy.Publisher("recording_started", Empty, queue_size=1)
        self.recording_finished_pub = rospy.Publisher("recording_finished", Empty, queue_size=1)
        #self.velocity_constraints_pub = rospy.Publisher('/robot1/velocity_constraints', Float64MultiArray, queue_size=1)

        #Timers
        self.record_data_heartbeat = rospy.Timer(rospy.Duration(0.25),self.record_data_heartbeat_callback)
        self.debug_heartbeat = rospy.Timer(rospy.Duration(2),self.debug_heartbeat_callback)

        #open files
        self.metrics_file           = open(self.test_name+str(self.current_experiment)+"_metrics.txt","w")
        self.robot_human_satus_file = open(self.test_name+str(self.current_experiment)+"_robot_human_status.txt","w")
        
        rospy.spin()

    def record_data_heartbeat_callback(self, timer):
        if self.recording:
            #calculte relative speed
            self.relative_robot_human_speed_in_x = self.robot_lin_x - self.closest_person_lin_x

            #save data at each heartbeat
            self.robot_human_satus_file.write(str(self.robot_lin_x) +","+ \
                                              str(self.robot_ang_z) +","+ \
                                              str(self.closest_person_x) +","+ \
                                              str(self.closest_person_y) +","+ \
                                              str(self.closest_person_yaw) +","+ \
                                              str(self.closest_person_lin_x) +","+ \
                                              str(self.closest_person_lin_y) +","+ \
                                              str(self.closest_human_distance) +","+ \
                                              str(self.relative_robot_human_speed_in_x) + "\n")

    def mission_status_callback(self, msg_data):  # this is used to trigger the end of the recording process
       
        mission_status = json.loads(msg_data.data)
        if mission_status["completed"] != [] and self.metrics_saved==False:
            rospy.loginfo("Recording has finished")
            self.recording=False
            self.total_mission_time = rospy.get_time() - self.starting_time

            #save the rest of the variables
            self.metrics_file.write("total mission time: " + str(self.total_mission_time) + "\n")
            self.metrics_file.write("brakes: "             + str(self.num_brakes) + "\n")
            self.metrics_file.write("vsmu constraints: "   + str(self.num_vsmu_constraints) + "\n")
            self.metrics_file.write("hri constraints: "    + str(self.num_hri_constraints) + "\n")
            self.metrics_file.write("hri replans: "        + str(self.num_hri_replans)  + "\n")

            self.metrics_file.close()
            self.robot_human_satus_file.close()
            self.metrics_saved = True

            self.recording_finished_pub.publish(Empty())

    def active_robot_status_callback(self, msg_data): # this is used to trigger the start of the recording process
        
        active_robots = json.loads(msg_data.data)

        if (len(active_robots) > 0):
            if self.recording == False:
                self.starting_time = rospy.get_time()
                rospy.loginfo("Recording has started")
                self.recording = True

                self.num_hri_replans = 0
                self.num_hri_constraints = 0
                self.num_vsmu_constraints = 0
                self.num_brakes = 0

                self.recording_started_pub.publish(Empty())


    def robot_odometry_callback(self, msg_data):
        self.robot_lin_x = msg_data.twist.twist.linear.x
        self.robot_ang_z = msg_data.twist.twist.angular.z  

    def tracked_persons_callback(self,msg_data): #checkl the closest human and save its information
        min_human_distance = 999999
        for track in msg_data.tracks:
            current_human_distance = math.sqrt(track.pose.pose.position.x*track.pose.pose.position.x + track.pose.pose.position.y*track.pose.pose.position.y)
            if current_human_distance < min_human_distance:
                self.closest_person_x = track.pose.pose.position.x
                self.closest_person_y = track.pose.pose.position.y
                r,p,self.closest_person_yaw = tf.transformations.euler_from_quaternion([track.pose.pose.orientation.x,track.pose.pose.orientation.y,track.pose.pose.orientation.z,track.pose.pose.orientation.w])
                self.closest_person_lin_x = track.twist.twist.linear.x
                self.closest_person_lin_y = track.twist.twist.linear.y
                self.closest_human_distance = current_human_distance

    def controller_command_callback(self, msg_data): # this is used to  count the number of brakes
        if self.old_controller_command != msg_data.command:
            self.old_controller_command = msg_data.command

            if msg_data.command == 1:
                self.num_brakes = self.num_brakes + 1


    def debug_heartbeat_callback(self, timer):
        rospy.loginfo("total mission time: " + str(rospy.get_time() - self.starting_time) + \
              " - emergency stops: "    + str(self.num_brakes) + \
              " - vsmu constraints: "   + str(self.num_vsmu_constraints) + \
              " - hri constraints: "    + str(self.num_hri_constraints) + \
              " - hri replans: "        + str(self.num_hri_replans) )

    def layer3_activated_callback(self, msg_data):
        self.num_hri_constraints = self.num_hri_constraints + 1

    def layer3_replan_triggered_callback(self,msg_data):
        self.num_hri_replans = self.num_hri_replans + 1

    def layer2_activated_callback(self,msg_data):
        self.num_vsmu_constraints = self.num_vsmu_constraints + 1


if __name__ == '__main__':
    rospy.init_node("iliad_safety_metrics_saver_node")#, log_level=rospy.DEBUG)
    try:
        isms = iliad_safey_metrics_saver()
        self.metrics_file.close()
    except rospy.ROSInterruptException:
        rospy.loginfo("Error -exception raised") 