#!/usr/bin/env python

'''
Connects to trajectory topic (from v.e.n.) and report (from mpc)
Creates a list of goals for dwa to track
(geometry_msgs::PoseStamped)
'''

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from orunav_msgs.msg import ControllerTrajectoryChunkVec, ControllerReport
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

import tf
from threading import Lock



class retriever():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params        
        self.path_mutex = Lock()
        self.next_chunk_step_list = []
        # ................................................................
        # start ros subs/pubs/servs....
        self.initROS()

        rospy.loginfo("Node [GoalRetriever] STARTED")
    
        rospy.spin()
    
    def loadROSParams(self):
        self.robot_id = rospy.get_param('~robot_id', 4) 
        self.goal_frame_id = rospy.get_param('~goal_frame_id', 'world') 

        # how many steps ahead we use for target
        self.look_ahead = rospy.get_param('~look_ahead', 5) 

        self.reports_topic = rospy.get_param('~reports_topic', '/robot4/control/controller/reports') 
        self.trajectory_topic = rospy.get_param('~trajectory_topic', '/robot4/control/controller/trajectories')
        self.goals_topic = rospy.get_param('~goals_topic', '/robot4/control/controller/nextGoal')
        self.path_topic = rospy.get_param('~path_topic', '/robot4/control/controller/path')
        self.status_topic = rospy.get_param('~status_topic', '/robot4/move_base/status')

    def initROS(self):
        self.goal_pub = rospy.Publisher(self.goals_topic, PoseStamped, queue_size=1)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1)
        rospy.Subscriber(self.trajectory_topic, ControllerTrajectoryChunkVec, self.trajectory_callback, queue_size=1)
        rospy.Subscriber(self.reports_topic, ControllerReport, self.reports_callback, queue_size=1)
        rospy.Subscriber(self.status_topic, GoalStatusArray, self.status_callback, queue_size=1)

    def trajectory_callback(self,msg):     
        '''
        When robot receives a trajectory list, we store it.
        '''
        self.path_mutex.acquire()   
        self.path = Path()
        self.path.header.frame_id = self.goal_frame_id
        self.next_chunk_step_list = []

        for chunk_ind in range(len(msg.chunks)):
            chunk = msg.chunks[chunk_ind]

            for goal_step_ind in range(len(chunk.steps)):                
                goal_step = chunk.steps[goal_step_ind]

                aPoseSt = PoseStamped()
                aPoseSt.header.frame_id = self.goal_frame_id
                aPoseSt.pose.position.x = goal_step.state.position_x
                aPoseSt.pose.position.y = goal_step.state.position_y
                
                quat = tf.transformations.quaternion_from_euler(0,0,goal_step.state.orientation_angle)
                aPoseSt.pose.orientation.x = quat[0]
                aPoseSt.pose.orientation.y = quat[1]
                aPoseSt.pose.orientation.z = quat[2]
                aPoseSt.pose.orientation.w = quat[3]                
                self.path.poses.append(aPoseSt)
                
                self.next_chunk_step_list.append((chunk_ind,goal_step_ind))


        self.path_pub.publish(self.path)    
        self.curr_goal_ind = 0
        self.curr_chunk_step = self.next_chunk_step_list[self.curr_goal_ind]
        self.path_mutex.release()   
        self.publishNextGoal()        

    def publishNextGoal(self):
        self.path_mutex.acquire()   

        next_goal = self.curr_goal_ind + self.look_ahead
        if next_goal > len(self.path.poses)-1:
            next_goal = len(self.path.poses)-1

        if not (self.curr_goal_ind == next_goal):
            self.goal_pub.publish(self.path.poses[next_goal])
            rospy.loginfo("I just published goal num: "+str(next_goal)+" from steps")
            self.curr_goal_ind = next_goal
            self.curr_chunk_step = self.next_chunk_step_list[next_goal]
        else:
            rospy.loginfo("Goal num: "+str(next_goal)+" already published. Skipping")

        self.path_mutex.release()   


    def status_callback(self,msg):
        pass
        # move base is a passenger... does not rule the robot
        #if (len(msg.status_list)>0) and (msg.status_list[-1].status == GoalStatus.SUCCEEDED):
        #    self.publishNextGoal()


    def reports_callback(self,msg):
        report_chunk_ind = msg.traj_chunk_sequence_num
        report_step_ind = msg.traj_step_sequence_num
        
        rospy.logdebug("I just reached chunk: "+str(report_chunk_ind)+", step: "+str(report_step_ind))
        try:
            if (report_chunk_ind,report_step_ind)==self.curr_chunk_step:
                self.publishNextGoal()
        except AttributeError:
            rospy.logdebug_throttle(10,"Dont have chunk_step, probably no trajectory stored")

# Main function.
if __name__ == '__main__':
        rospy.init_node('GoalRetriever')#, log_level=rospy.DEBUG)
        # Go to class functions that do all the heavy lifting. Do error checking.
        try:
            goGo=retriever()
        except rospy.ROSInterruptException:
            pass

