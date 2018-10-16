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



class Halp():

    # Must have __init__(self) function for a class, similar to a C++ class constructor.
    def __init__(self):

        # ................................................................
        # read ros parameters
        self.loadROSParams()

        # ................................................................
        # Other config params        
        self.path_mutex = Lock()
        # dictionary indexed by global trajectories id, containing paths with all steps from chunks in given trajectory
        self.paths = dict()
        # as above, but containing list of global chunk,step tuples corresponding each path point
        self.chunk_step_lists = dict()
        # as above, but containing list of global chunk,step tuples corresponding each path point
        self.local_to_global_mapping = dict()

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

        # for debugging
        self.goals_topic = rospy.get_param('~goals_topic', '/robot'+str(self.robot_id)+'/control/controller/nextGoal')
        # for debugging
        self.path_topic = rospy.get_param('~path_topic', '/robot'+str(self.robot_id)+'/control/controller/path')

        self.in_command_topic = rospy.get_param('~in_command_topic', '/robot'+str(self.robot_id)+'/control/controller/commands_ven')
        self.out_command_topic = rospy.get_param('~out_command_topic', '/robot'+str(self.robot_id)+'/control/controller/commands_mpc')

        self.in_reports_topic = rospy.get_param('~in_reports_topic', '/robot'+str(self.robot_id)+'/control/controller/reports_mpc') 
        self.out_reports_topic = rospy.get_param('~out_reports_topic', '/robot'+str(self.robot_id)+'/control/controller/reports_ven') 

        self.in_trajectory_topic = rospy.get_param('~in_trajectory_topic', '/robot'+str(self.robot_id)+'/control/controller/trajectories_ven')
        self.out_trajectory_topic = rospy.get_param('~out_trajectory_topic', '/robot'+str(self.robot_id)+'/control/controller/trajectories_mpc')


    def initROS(self):
        # only for debug purposes
        self.goal_pub = rospy.Publisher(self.goals_topic, PoseStamped, queue_size=1)
        # only for debug purposes
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1)

        self.trajectory_pub = rospy.Publisher(self.out_trajectory_topic, ControllerTrajectoryChunkVec, queue_size=1)
        self.command_pub = rospy.Publisher(self.out_command_topic, ControllerCommand, queue_size=1)
        self.reports_pub = rospy.Publisher(self.out_reports_topic, ControllerReport, queue_size=1)

        rospy.Subscriber(self.in_trajectory_topic, ControllerTrajectoryChunkVec, self.trajectory_callback, queue_size=1)
        rospy.Subscriber(self.in_command_topic, ControllerCommand, self.command_callback, queue_size=1)
        rospy.Subscriber(self.in_reports_topic, ControllerReport, self.reports_callback, queue_size=1)



'''
    We receive a oru_trajectory vector from vehicle execution node:
    - build a Path from trajectory. 
    - build a mapping between (chunk,step) of input trajectory and path poses
    - If we have that trajectory as active, start transmititng
'''
    def trajectory_callback(self,msg):     
        self.path_mutex.acquire()   
        path = Path()
        self.path.header.frame_id = self.goal_frame_id
        chunk_step_list = []

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
                path.poses.append(aPoseSt)
                
                chunk_step_list.append((chunk_ind,goal_step_ind))

        self.paths[msg.chunks[0].traj_id] = path
        self.chunk_step_lists[msg.chunks[0].traj_id] = chunk_step_list 



        # debug...
        self.path_pub.publish(self.path)    
        self.curr_goal_ind = 0
        self.curr_chunk_step = self.chunk_step_list[self.curr_goal_ind]
        self.path_mutex.release()   

'''
    We receive a command from vehicle execution node. Possible commands:
    COMMAND_BRAKE     => forward
    COMMAND_ACTIVATE  => get ready for a global trajectory arrival
    COMMAND_STARTTIME => forward 
    COMMAND_RECOVER   => forward
'''
    def command_callback(self,msg):
        rospy.logdebug("Received command: "+self.printCommand(msg.command))
        if (msg.command == msg.COMMAND_ACTIVATE):
            if self.robot_id == msg.robot_id:
                self.setGlobalTrajectory(msg.traj_id)
        else            
            self.out_command_pub.publish(msg)
    

'''
    MPC updates its position and reports locally executed (chunk,step).
    We "transform" that into the equivalent (chunk,step) for ven,  
    by publishing a report changing step number.
    
    After that, we continue publishing Trajectories.
'''
    def reports_callback(self,msg):
        local_chunk_ind = msg.traj_chunk_sequence_num
        local_step_ind = msg.traj_step_sequence_num
        
        rospy.logdebug("Controller status is: "+self.printStatus(msg.status))
        rospy.logdebug("I just reached:")
        rospy.logdebug("    local (chunk,step)==("+str(local_chunk_ind)+",  "+str(local_step_ind)+")")


        # Trajectory sequence numbers, only valid if the controller status is active
        if (msg.status == ControllerReport.CONTROLLER_STATUS_ACTIVE):        
            (global_chunk_sequence_num, global_step_sequence_num) = self.mapChunkStep(local_chunk_sequence_num, local_step_sequence_num)
            msg.traj_chunk_sequence_num = global_chunk_ind
            msg.traj_step_sequence_num = global_step_ind
            rospy.logdebug("    global (chunk,step)==("+str(global_chunk_ind)+",  "+str(global_step_ind)+")")            

        if (msg.status == ControllerReport.CONTROLLER_STATUS_WAIT) and (self.moreToTransmit()):                    
            self.commandNextLocalTrajectory()
       
        self.reports_pub.publish(msg)


    '''
        Define new global trajectory id to chop and send down.
    '''
    def setGlobalTrajectory(self, traj_id):
        TODO

    '''
        Returnt true if there is more goals to 
    '''
    def moreToTransmit(self):
        TODO!

    def mapChunkStep(self, local_chunk_seq, local_step_seq):
        TODO!

    '''
        - Retrieve (look_ahead steps ahed) goal from our stored path
        - Ask service getPlan to build a path
        - Build new oru_trajectory based on that path
        - Send command activating this trajectory
        - Send trajectory itself.
    '''
    def commandNextLocalTrajectory(self):
        TODO!

    def publishNextGoal(self):
            self.path_mutex.acquire()   

            next_goal = self.curr_goal_ind + self.look_ahead
            if next_goal >= len(self.path.poses):
                next_goal = len(self.path.poses)-1

            if not (self.curr_goal_ind == next_goal):
                self.goal_pub.publish(self.path.poses[next_goal])
                rospy.loginfo("I just published goal num: "+str(next_goal)+" from steps")
                self.curr_goal_ind = next_goal
                self.curr_chunk_step = self.chunk_step_list[next_goal]
            else:
                rospy.loginfo("Goal num: "+str(next_goal)+" already published. Skipping")

            self.path_mutex.release()   





'''
Accessory functions
'''
def printStatus(self,statusInt):
    switcher = {
        ControllerReport.CONTROLLER_STATUS_WAIT: "WAIT",
        ControllerReport.CONTROLLER_STATUS_FAIL: "FAIL",
        ControllerReport.CONTROLLER_STATUS_ACTIVE: "ACTIVE",
        ControllerReport.CONTROLLER_STATUS_FINALIZE: "FINALIZE",
        ControllerReport.CONTROLLER_STATUS_TERMINATE: "TERMINATE"
    }
    return switcher.get(statusInt, "Unknown state")

def printCommand(self,commandInt):
    switcher = {
        ControllerCommand.COMMAND_BRAKE: "BRAKE",
        ControllerCommand.COMMAND_ACTIVATE: "ACTIVATE",
        ControllerCommand.COMMAND_STARTTIME: "STARTTIME",
        ControllerCommand.COMMAND_RECOVER: "RECOVER",
    }
    return switcher.get(commandInt, "Unknown command")


# Main function.
if __name__ == '__main__':
        rospy.init_node('halp_node')#, log_level=rospy.DEBUG)
        # Go to class functions that do all the heavy lifting. Do error checking.
        try:
            goGo=retriever()
        except rospy.ROSInterruptException:
            pass

