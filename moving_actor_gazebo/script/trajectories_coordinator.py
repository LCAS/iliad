#!/usr/bin/env python

import rospy
import numpy as np
import json
import roslaunch
import rosnode
from rosgraph_msgs.msg import Clock

class trajectories_coordinator(object):

	def __init__(self):
		#parameters
		self.trajectories_file = rospy.get_param('~trajectories_file',"my_trajectories_example.txt")		
		self.num_available_actors = rospy.get_param('~num_available_actors',2)
		self.relative_times = rospy.get_param('~relative_times',1)
		self.actor_trajectory_path = rospy.get_param('~actor_trajectory_path',"$(find moving_actor_gazebo)/script/")
		
		self.run()


	def execute_trajecory(self,trajectory_to_execute):
		print('execute_trajectory:'+str(trajectory_to_execute))

		#find if there is an actor available to execute the trajectory
		if np.sum(self.actor_working)<self.num_available_actors:
			print("Actor available!")
			for a in range(0, self.num_available_actors):
				if self.actor_working[a]==0:
					actor_to_execute = a
					self.actor_working[a] = 1
					break


			# copy the trajectory to the file "actorXX.json"
			my_json_trajectory = json.loads(self.my_trajectories[trajectory_to_execute])
			with open(self.actor_trajectory_path+'actor0'+str(actor_to_execute)+'.json','w') as jfile:
				json.dump(my_json_trajectory,jfile)

			actor_name = "actor0"+str(actor_to_execute)
			actor_trajectory_file = self.actor_trajectory_path+actor_name+".json"
			global_frame_id = "world"


			new_goal_feeder_node = roslaunch.core.Node("moving_actor_gazebo","goal_feeder.py",name="goal_feeder_actor0"+str(actor_to_execute),output="log",args=\
			    " _model_name:="+actor_name+\
			    " _goals_filename:="+actor_trajectory_file+\
			    " _frame_id:="+global_frame_id+\
			    " _robot_base_frame_id:="+actor_name+"/base_link"+\
			    " _move_base_actionserver_name:=/"+actor_name+"/move_base"+\
			    " _loop:=False"+\
			    " _visual_pub_topic_name:=/"+actor_name+"/markers"\
				)

			launch = roslaunch.scriptapi.ROSLaunch()
			launch.start()
			process = launch.launch(new_goal_feeder_node)
			self.actor_processes[actor_to_execute] = process

		else:
			print("No available actors - Skipping trajectory: "+str(trajectory_to_execute))


	def run(self):
		# Read trajectories file
		self.trajectories_times = np.loadtxt(self.trajectories_file,delimiter=";",dtype="float",usecols=0)
		self.my_trajectories = np.loadtxt(self.trajectories_file,delimiter=";",dtype="string",usecols=1)
		while rospy.get_time()==0:
			pass

		if self.relative_times:
			self.trajectories_times = self.trajectories_times + rospy.get_time()

		
		self.actor_working = np.zeros(self.num_available_actors)
		self.actor_processes = {}

		current_trajectory = 0
		all_trajectories_done = 0
		r = rospy.Rate(10)
		while (not rospy.is_shutdown()):
			#handle the execution of the trajectories
			if self.trajectories_times[current_trajectory] <= rospy.get_time():

				if all_trajectories_done == 0:
					self.execute_trajecory(current_trajectory)

				if current_trajectory + 1 == len(self.trajectories_times):
					all_trajectories_done = 1
				else:
					current_trajectory = current_trajectory + 1

			#handle the ending of the trajectories
			for i in range(0,self.num_available_actors):
				if self.actor_working[i] == 1:
					if self.actor_processes[i].is_alive() == False:
						self.actor_working[i] = 0

				if np.sum(self.actor_working)==0 and all_trajectories_done==1:
					print("All trajectories completed")
					return
			
			print("Actors working:",self.actor_working)
			r.sleep()



if __name__ == '__main__':
	rospy.init_node('trajectories_coordinator_node', anonymous=True)
	try:
		tc = trajectories_coordinator()
	except rospy.ROSInterruptException:
		pass