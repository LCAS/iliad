#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose,PoseStamped
import Tkinter as tk
from std_msgs.msg import String, Int16
import tkMessageBox
import xml.etree.ElementTree as ET
import json
from orunav_msgs.msg import RobotReport
import tf.transformations

class iliad_goal_manager(object):

	def __init__(self):
		#parameters
		self.orders_file = rospy.get_param('~orders_file',"")
		self.missions_file = rospy.get_param('~missions_file',"")
		self.orders_times_file = rospy.get_param('~orders_times_file',"")
		self.items_locations_file = rospy.get_param('~items_locations_file',"")
		self.locations_coordinates_file = rospy.get_param('~locations_coordinates_file',"")
		self.locations_frame_id = rospy.get_param('~locations_frame_id',"world")
		self.mode = rospy.get_param('~mode',0) # [0]: click and point, [1]: missions 

		# ini variables
		self.missions_started = 0
		self.available_robots = {}
		self.active_robots = {}
		self.next_mission = 0
		self.queued_missions = []
		self.all_missions_added = 0
		self.completed_missions = []
		self.mission_status = {"completed":[],"queued":[],"seconds_left_next":0}
		self.shutdown_node = False
		self.exploration_ongoing = False
		self.robot_navigation_status = {}
		

		if self.mode==0: #click and point 
			print "GOAL MANAGER STARTED WITH POINT AND CLICK MODE"
			#subscribers
			rospy.Subscriber("/robot1/point_click_goal", PoseStamped, self.robot1_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot2/point_click_goal", PoseStamped, self.robot2_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot3/point_click_goal", PoseStamped, self.robot3_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot4/point_click_goal", PoseStamped, self.robot4_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot5/point_click_goal", PoseStamped, self.robot5_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot6/point_click_goal", PoseStamped, self.robot6_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot7/point_click_goal", PoseStamped, self.robot7_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot8/point_click_goal", PoseStamped, self.robot8_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot9/point_click_goal", PoseStamped, self.robot9_pointclickgoal_callback,queue_size=1)

			# publishers
			self.robot1_goal_pub = rospy.Publisher("/robot1/goal", PoseStamped,queue_size=1)
			self.robot2_goal_pub = rospy.Publisher("/robot2/goal", PoseStamped,queue_size=1)
			self.robot3_goal_pub = rospy.Publisher("/robot3/goal", PoseStamped,queue_size=1)
			self.robot4_goal_pub = rospy.Publisher("/robot4/goal", PoseStamped,queue_size=1)
			self.robot5_goal_pub = rospy.Publisher("/robot5/goal", PoseStamped,queue_size=1)
			self.robot6_goal_pub = rospy.Publisher("/robot6/goal", PoseStamped,queue_size=1)
			self.robot7_goal_pub = rospy.Publisher("/robot7/goal", PoseStamped,queue_size=1)
			self.robot8_goal_pub = rospy.Publisher("/robot8/goal", PoseStamped,queue_size=1)
			self.robot9_goal_pub = rospy.Publisher("/robot9/goal", PoseStamped,queue_size=1)

			rospy.spin()

		else: # missions mode
			# subscribe to topics
			rospy.Subscriber("/exploration_goal", PoseStamped, self.exploration_goal_callback,queue_size=1)
			rospy.Subscriber("/robot1/control/report", RobotReport, self.robot1_status_callback,queue_size=1)
			rospy.Subscriber("/robot2/control/report", RobotReport, self.robot2_status_callback,queue_size=1)
			rospy.Subscriber("/robot3/control/report", RobotReport, self.robot3_status_callback,queue_size=1)
			rospy.Subscriber("/robot4/control/report", RobotReport, self.robot4_status_callback,queue_size=1)
			rospy.Subscriber("/robot5/control/report", RobotReport, self.robot5_status_callback,queue_size=1)
			rospy.Subscriber("/robot6/control/report", RobotReport, self.robot6_status_callback,queue_size=1)
			rospy.Subscriber("/robot7/control/report", RobotReport, self.robot7_status_callback,queue_size=1)
			rospy.Subscriber("/robot8/control/report", RobotReport, self.robot8_status_callback,queue_size=1)
			rospy.Subscriber("/robot9/control/report", RobotReport, self.robot9_status_callback,queue_size=1)

			# create topic publishers
			self.active_robots_status_pub = rospy.Publisher('/active_robot_status', String,queue_size=10)
			self.mission_status_pub = rospy.Publisher('/mission_status', String,queue_size=10)
			self.robot1_goal_pub = rospy.Publisher("/robot1/goal", PoseStamped,queue_size=1)
			self.robot2_goal_pub = rospy.Publisher("/robot2/goal", PoseStamped,queue_size=1)
			self.robot3_goal_pub = rospy.Publisher("/robot3/goal", PoseStamped,queue_size=1)
			self.robot4_goal_pub = rospy.Publisher("/robot4/goal", PoseStamped,queue_size=1)
			self.robot5_goal_pub = rospy.Publisher("/robot5/goal", PoseStamped,queue_size=1)
			self.robot6_goal_pub = rospy.Publisher("/robot6/goal", PoseStamped,queue_size=1)
			self.robot7_goal_pub = rospy.Publisher("/robot7/goal", PoseStamped,queue_size=1)
			self.robot8_goal_pub = rospy.Publisher("/robot8/goal", PoseStamped,queue_size=1)
			self.robot9_goal_pub = rospy.Publisher("/robot9/goal", PoseStamped,queue_size=1)
			self.exploration_allowed_pub = rospy.Publisher("exploration_allowed",Int16,queue_size=1)
			


			# create timers
			self.assign_missions_and_goals_timer = rospy.Timer(rospy.Duration(0.5),self.assign_missions_and_goals)
			
			#parse input files
			# read orders file
			if self.orders_file != "":
				self.missions = self.parse_orders_file()
				self.missions_file = ""
			elif self.missions_file != "":
				self.missions = self.parse_missions_file()
			else:
				print "NO MISSIONS BEING PROVIDED - CLOSING NODE"
				return

			#print "\n",self.missions
			# Exmple of missions variable structure with 4 simple orders
			#self.test_missions = [["pick:soup","go:home"], ["pick:soup","go:home"]]
			#print "\n",self.test_missions

			self.parse_orders_times_file()

			if len(self.missions) != len(self.missions_times):
				print "NUM MISSIONS IS DIFFERENT FROM NUM MISSION TIMES PROVIDED - CLOSING NODE"
				return

			self.parse_item_locations_file()
			self.parse_location_coordinates_file()
			
			#start gui variables		
			self.start_gui()
			
			# ini the main loop
			self.run()

	def parse_orders_file(self):
		# the input is an xml file with all the orders to be completed in a day
		missions = []
		mission= []		
		orders = ET.parse(self.orders_file).getroot()
		print "Num of orders: "+str(len(orders))

		for o in range(0,len(orders)):
			print ""
			print "> Order "+str(o)
			for pallet_type in range(0,len(orders[o])):
				if orders[o][pallet_type].tag == "FullPallets" and len(orders[o][pallet_type]) > 0:
					print "--> Num of full pallets: " + str(len(orders[o][pallet_type]))
					for pallet in range(0,len(orders[o][pallet_type])):
						print "----> Pallet "+str(pallet)
						item_name = orders[o][pallet_type][pallet][0][0].attrib["name"]
						print "------> Item: "+item_name
						mission.append("go:fullpallet_"+item_name)
						mission.append("pick:fullpallet_"+item_name)
						
						mission.append("go:shipping_area")
						mission.append("drop:pallet")

				else:
					print "--> Num of full pallets: 0"

				if orders[o][pallet_type].tag == "MixedPallets" and len(orders[o][pallet_type]) > 0:
					print "--> Num of mixed pallets: " + str(len(orders[o][pallet_type]))
					for pallet in range(0,len(orders[o][pallet_type])):
						print "----> Pallet "+str(pallet)
						previous_item_name = ""
						for item in range(0,len(orders[o][pallet_type][pallet][0])):
							item_name = orders[o][pallet_type][pallet][0][item].attrib["name"]
							print "------> Item "+str(item)+": "+item_name
							if item == 0:
								mission.append("go:empty_pallet")
								mission.append("pick:empty_pallet")

							if item_name != previous_item_name:
								mission.append("go:"+item_name)
								mission.append("pick:"+item_name)
							else:
								mission.append("pick:"+item_name)
							previous_item_name = item_name

						mission.append("go:shipping_area")
						mission.append("drop:pallet")
				else:
					print "--> Num of mixed pallets: 0"

			mission.append("go:home")
			missions.append(mission)
			mission = []
		return missions

	def parse_missions_file(self):
		# the inputs is a string containing all
		missions = []
		with open(self.missions_file,"r") as file:
			for line in file:
				if line[-1] == '\n':
					mission = line[:-1].split(",")
				else:
					mission = line.split(",")

				missions.append(mission)
		return missions

	def parse_orders_times_file(self):
		self.missions_times = []
		with open(self.orders_times_file,"r") as file:
			for line in file:
				self.missions_times.append(int(line))
		#print self.missions_times

	def parse_item_locations_file(self):
		with open(self.items_locations_file,"r") as file:
			self.item_locations_data = json.load(file)
		#print self.item_locations_data

	def parse_location_coordinates_file(self):
		with open(self.locations_coordinates_file,"r") as file:
			self.location_coordinates_data = json.load(file)

	def window_closing(self):
		self.gui.destroy()
		self.shutdown_node = True
		return

	def start_gui(self):
		#start the self.gui
		self.gui = tk.Tk()
		self.gui.title("ILIAD Goal manager")
		self.gui.protocol("WM_DELETE_WINDOW",self.window_closing)

		self.mission_mode_frame = tk.LabelFrame(self.gui,text="MISSION MODE",font=("Arial Bold", 12))
		self.mission_mode_frame.grid(row=0,column=1,padx=10, pady=10,sticky="w")
		
		self.files_frame = tk.LabelFrame(self.mission_mode_frame,text="Files Loaded",font=("Arial Bold", 12))
		self.files_frame.grid(row=0,column=0,padx=10, pady=10,sticky="w")

		self.missions_frame = tk.LabelFrame(self.mission_mode_frame,text="Mission Status",font=("Arial Bold", 12))
		self.missions_frame.grid(row=1,column=0,padx=10, pady=10,sticky="w")

		self.robots_frame = tk.LabelFrame(self.mission_mode_frame,text="Robot Status",font=("Arial Bold", 12))
		self.robots_frame.grid(row=2,column=0,padx=10, pady=10,sticky="w")

		#files frame
		orders_label = tk.Label(self.files_frame, text = "Orders: ",font=("Arial Bold", 10))
		missions_label = tk.Label(self.files_frame, text = "Missions: ",font=("Arial Bold", 10))
		orders_times_label = tk.Label(self.files_frame, text = "Orders/Misions times: ",font=("Arial Bold", 10)) 
		items_locations_label = tk.Label(self.files_frame, text = "Items locations: ",font=("Arial Bold", 10))
		locations_coordinates_label = tk.Label(self.files_frame, text = "Locations coordinates: ",font=("Arial Bold", 10))
		orders_label.grid(               row=0,column=0,pady=3,padx=5,sticky="e")
		missions_label.grid(             row=1,column=0,pady=3,padx=5,sticky="e")
		orders_times_label.grid(         row=2,column=0,pady=3,padx=5,sticky="e")
		items_locations_label.grid(      row=3,column=0,pady=3,padx=5,sticky="e")
		locations_coordinates_label.grid(row=4,column=0,pady=3,padx=5,sticky="e")

		orders_text = tk.Label(self.files_frame, text = self.orders_file,font=("Arial Bold", 10),bg="white",width=100)
		missions_text = tk.Label(self.files_frame, text = self.missions_file,font=("Arial Bold", 10),bg="white",width=100)
		orders_times_text = tk.Label(self.files_frame, text = self.orders_times_file,font=("Arial Bold", 10),bg="white",width=100)
		items_locations_text = tk.Label(self.files_frame,       text = self.items_locations_file,font=("Arial Bold", 10),bg="white",width=100)
		locations_coordinates_text = tk.Label(self.files_frame, text = self.locations_coordinates_file,font=("Arial Bold", 10),bg="white",width=100)
		orders_text.grid(               row=0,column=1,sticky="w")
		missions_text.grid(             row=1,column=1,sticky="w")
		orders_times_text.grid(         row=2,column=1,sticky="w")
		items_locations_text.grid(      row=3,column=1,sticky="w")
		locations_coordinates_text.grid(row=4,column=1,sticky="w")

		# mission status panel
		available_label = tk.Label(self.missions_frame, text = "Robots available ",font=("Arial Bold", 10),justify="right")
		available_label.grid(row=0,column=0,pady=5,padx=5,columnspan=2)

		reload_list_button = tk.Button(self.missions_frame, text ="Reload\nlist", command = self.reload_available_robots_callback, width=5,background="orange",activebackground="orange",font=("Arial Bold", 10))
		reload_list_button.grid(row=1,rowspan=3,column = 0,padx=5)

		self.available_list = tk.Listbox(self.missions_frame,width=15,font=("Arial Bold", 10),height=4,justify="center",selectmode="multiple",selectbackground="green")
		self.available_list.grid(row=1,column=1,rowspan=3,padx=5,pady=5)

		self.allow_exploration = tk.IntVar()
		allow_exploration_checkbutton = tk.Checkbutton(self.missions_frame,text="Allow exploration",variable=self.allow_exploration,onvalue=1,offvalue=0,font=("Arial Bold", 10))
		allow_exploration_checkbutton.grid(row=4,column=1,padx=5,pady=5)

		start_missions_button = tk.Button(self.missions_frame, text ="Start\nmissions", command = self.start_missions_callback,height=5, width=5,background="green",activebackground="green",font=("Arial Bold", 10))
		start_missions_button.grid(row=1,rowspan=4,column = 2,padx=5)

		progress_label = tk.Label(self.missions_frame, text = "Progress: ",font=("Arial Bold", 10),justify="right")
		completed_label = tk.Label(self.missions_frame, text = "Completed: ",font=("Arial Bold", 10),justify="right")
		queued_label = tk.Label(self.missions_frame, text = "Queued: ",font=("Arial Bold", 10),justify="right")
		next_label =      tk.Label(self.missions_frame, text = "Next: ",font=("Arial Bold", 10),justify="right")
		progress_label.grid( row=1,column=3,sticky="e",pady=3,padx=5)
		completed_label.grid(row=2,column=3,sticky="e",pady=3,padx=5)
		queued_label.grid(   row=3,column=3,sticky="e",pady=3,padx=5)
		next_label.grid(     row=4,column=3,sticky="e",pady=3,padx=5)

		self.progress_text_label = tk.Label(self.missions_frame,font=("Arial Bold", 10),bg="white",width = 35)
		self.completed_text_label = tk.Label(self.missions_frame,font=("Arial Bold", 10),bg="white",width = 35)
		self.queued_text_label = tk.Label(self.missions_frame,font=("Arial Bold", 10),bg="white",width = 35)
		self.next_text_label =      tk.Label(self.missions_frame,font=("Arial Bold", 10),bg="white",width = 35)
		self.progress_text_label.grid( row=1,column=4,sticky="w")
		self.completed_text_label.grid(row=2,column=4,sticky="w")
		self.queued_text_label.grid(   row=3,column=4,sticky="w")
		self.next_text_label.grid(     row=4,column=4,sticky="w")

		abort_missions_button = tk.Button(self.missions_frame, text ="Abort\nall\nmissions", command = self.abort_missions_callback,height=5, width=5,background="red",activebackground="red",font=("Arial Bold", 10))
		abort_missions_button.grid(row=1,rowspan=4,column = 5,padx=5)

		# robot status panel
		active_label = tk.Label(self.robots_frame, text = "Active",font=("Arial Bold", 10))
		status_label = tk.Label(self.robots_frame, text = "Status",font=("Arial Bold", 10))
		mission_label = tk.Label(self.robots_frame, text = "Mission",font=("Arial Bold", 10))
		navigation_label = tk.Label(self.robots_frame, text = "Navigation",font=("Arial Bold", 10))
		wait_label = tk.Label(self.robots_frame, text = "Wait",font=("Arial Bold", 10))
		goal_label = tk.Label(self.robots_frame, text = "Goal",font=("Arial Bold", 10))
		action_label = tk.Label(self.robots_frame, text = "Action",font=("Arial Bold", 10))
		active_label.grid(row=0,column=0,pady=5,padx=5)
		status_label.grid(row=0,column=1,pady=5,padx=5)
		mission_label.grid(row=0,column=2,pady=5,padx=5)
		navigation_label.grid(row=0,column=3,pady=5,padx=5)
		wait_label.grid(row=0,column=4,pady=5,padx=5)
		goal_label.grid(row=0,column=5,pady=5,padx=5)
		action_label.grid(row=0,column=6,pady=5,padx=5)

		self.active_list = tk.Listbox(self.robots_frame,width=15,font=("Arial Bold", 10),height=4,justify="center")
		self.status_list = tk.Listbox(self.robots_frame,width=6,font=("Arial Bold", 10), height=4,justify="center")
		self.mission_list = tk.Listbox(self.robots_frame,width=7,font=("Arial Bold", 10),height=4,justify="center")
		self.navigation_list = tk.Listbox(self.robots_frame,width=10,font=("Arial Bold", 10),height=4,justify="center")
		self.wait_list = tk.Listbox(self.robots_frame,width=5,font=("Arial Bold", 10),height=4,justify="center")
		self.goal_list = tk.Listbox(self.robots_frame,width=12,font=("Arial Bold", 10),height=4,justify="center")
		self.action_list = tk.Listbox(self.robots_frame,width=30,font=("Arial Bold", 10),height=4,justify="center")
		self.active_list.grid(row=1,column=0,padx=5,pady=5)
		self.status_list.grid(row=1,column=1,padx=5,pady=5)
		self.mission_list.grid(row=1,column=2,padx=5,pady=5)
		self.navigation_list.grid(row=1,column=3,padx=5,pady=5)
		self.wait_list.grid(row=1,column=4,padx=5,pady=5)
		self.goal_list.grid(row=1,column=5,padx=5,pady=5)
		self.action_list.grid(row=1,column=6,padx=5,pady=5)

		skip_goal_button = tk.Button(self.robots_frame, text ="Skip\ncurrent\ngoal", command = self.skip_goal_callback, width=5,background="orange",activebackground="orange",font=("Arial Bold", 10))
		skip_mission_button = tk.Button(self.robots_frame, text ="Skip\ncurrent\nmission", command = self.skip_mission_callback, width=5,background="red",activebackground="red",font=("Arial Bold", 10))
		skip_goal_button.grid(row=1,column = 7,padx=5)
		skip_mission_button.grid(row=1,column = 8,padx=5)

		#self.gui.mainloop() #blocking

	def start_missions_callback(self):
		print "starting the missions"
		if self.missions_started == 0:
			#update the active robots list from the selection
			robot_names = self.available_list.get(0,tk.END)
			robot_selection =  self.available_list.curselection()

			if len(robot_selection) > 0:
				for i in range(0,len(robot_selection)):
					self.active_list.insert(tk.END,robot_names[robot_selection[i]])
					self.active_robots[robot_names[robot_selection[i]]] = {"status":"-","mission":"-","goal":"-","wait":0,"navigation":"-","action":"-"}

				# make the times relative to the starting time
				current_time = rospy.get_time()
				self.corrected_missions_times = self.missions_times[:]
				for t in range(0,len(self.missions_times)):
					self.corrected_missions_times[t] = self.missions_times[t] + current_time

				self.missions_started = 1

			else:
				tkMessageBox.showwarning("Warning","Select at least 1 robot\nfrom the available list\nto perform the missions")
				return
			
		return

	def abort_missions_callback(self):
		print "aborting all missions"
		if self.missions_started:
			self.available_list_updated = 0
			self.missions_started = 0
			self.available_robots = {}
			self.active_robots = {}
			self.next_mission = 0
			self.queued_missions = []
			self.all_missions_added = 0
			self.completed_missions = []

			# for robot in self.active_robots:
			# 	if self.active_robots[robot_selected]["mission"] != "-":
			# 		self.active_robots[robot_selected]["goal"] = "-"
			# 		self.active_robots[robot_selected]["mission"] = "-"
			# 		self.active_robots[robot_selected]["wait"] = 0


		return

	def skip_goal_callback(self):
		if len(self.active_list.curselection()) > 0:
			robot_names = self.active_list.get(0,tk.END)
			robot_selected  = robot_names[self.active_list.curselection()[0]]
			print "skippin goal in robot: ", robot_selected

			if self.active_robots[robot_selected]["mission"] != "-":
				self.process_new_goal(robot_selected)
		else:
			tkMessageBox.showwarning("Warning","Select at least 1 robot\nfrom the active list")

		return

	def skip_mission_callback(self):
		if len(self.active_list.curselection()) > 0:
			robot_names = self.active_list.get(0,tk.END)
			robot_selected  = robot_names[self.active_list.curselection()[0]]
			print "skippin mission in robot: ", robot_selected
			
			if self.active_robots[robot_selected]["mission"] != "-":
				self.completed_missions.append(self.active_robots[robot_selected]["mission"] )
				self.active_robots[robot_selected]["goal"] = "-"
				self.active_robots[robot_selected]["mission"] = "-"
				self.active_robots[robot_selected]["wait"] = 0
		else:
			tkMessageBox.showwarning("Warning","Select at least 1 robot\nfrom the active list")

		return

	def exploration_goal_callback(self,exploration_goal_msg):
		print "Exploration goal request received"
		already_exploring = 0
		for robot in self.active_robots:
			if self.active_robots[robot]["action"] == "EXPLORING":
				already_exploring = 1
				print "Robot already exploring"
				break
		
		if self.allow_exploration_value and already_exploring==0:
			for robot in self.active_robots:
				if self.active_robots[robot]["status"] == "FREE":
					#send an exploration to the free robot and set variable as exploration ongoing
					self.active_robots[robot]["action"] = "EXPLORING"
					self.active_robots[robot]["wait"] = rospy.get_time() + 150 #after 120 sec the exploring tag is removed

					if robot == "robot1":
						self.robot1_goal_pub.publish(exploration_goal_msg)
					elif robot == "robot2":
						self.robot2_goal_pub.publish(exploration_goal_msg)
					elif robot == "robot3":
						self.robot3_goal_pub.publish(exploration_goal_msg)
					elif robot == "robot4":
						self.robot4_goal_pub.publish(exploration_goal_msg)
					elif robot == "robot5":
						self.robot5_goal_pub.publish(exploration_goal_msg)
					elif robot == "robot6":
						self.robot6_goal_pub.publish(exploration_goal_msg)
					elif robot == "robot7":
						self.robot7_goal_pub.publish(exploration_goal_msg)
					elif robot == "robot8":
						self.robot8_goal_pub.publish(exploration_goal_msg)
					elif robot == "robot9":
						self.robot9_goal_pub.publish(exploration_goal_msg)

					self.exploration_ongoing = True
					break
		else:
			print "Exploration not allowed"
		return

	def robot1_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_navigation_status["robot1"] = "FREE"
		else:
			self.robot_navigation_status["robot1"] = "NAVIGATING"

		if self.missions_started:
			for robot in self.active_robots:
				self.active_robots[robot]["navigation"] = self.robot_navigation_status[robot]

	def robot2_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_navigation_status["robot2"] = "FREE"
		else:
			self.robot_navigation_status["robot2"] = "NAVIGATING"

		if self.missions_started:
			for robot in self.active_robots:
				self.active_robots[robot]["navigation"] = self.robot_navigation_status[robot]

	def robot3_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_navigation_status["robot3"] = "FREE"
		else:
			self.robot_navigation_status["robot3"] = "NAVIGATING"

		if self.missions_started:
			for robot in self.active_robots:
				self.active_robots[robot]["navigation"] = self.robot_navigation_status[robot]

	def robot4_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_navigation_status["robot4"] = "FREE"
		else:
			self.robot_navigation_status["robot4"] = "NAVIGATING"

		if self.missions_started:
			for robot in self.active_robots:
				self.active_robots[robot]["navigation"] = self.robot_navigation_status[robot]

	def robot5_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_navigation_status["robot5"] = "FREE"
		else:
			self.robot_navigation_status["robot5"] = "NAVIGATING"

		if self.missions_started:
			for robot in self.active_robots:
				self.active_robots[robot]["navigation"] = self.robot_navigation_status[robot]

	def robot6_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_navigation_status["robot6"] = "FREE"
		else:
			self.robot_navigation_status["robot6"] = "NAVIGATING"

		if self.missions_started:
			for robot in self.active_robots:
				self.active_robots[robot]["navigation"] = self.robot_navigation_status[robot]

	def robot7_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_navigation_status["robot7"] = "FREE"
		else:
			self.robot_navigation_status["robot7"] = "NAVIGATING"

		if self.missions_started:
			for robot in self.active_robots:
				self.active_robots[robot]["navigation"] = self.robot_navigation_status[robot]

	def robot8_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_navigation_status["robot8"] = "FREE"
		else:
			self.robot_navigation_status["robot8"] = "NAVIGATING"

		if self.missions_started:
			for robot in self.active_robots:
				self.active_robots[robot]["navigation"] = self.robot_navigation_status[robot]

	def robot9_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_navigation_status["robot9"] = "FREE"
		else:
			self.robot_navigation_status["robot9"] = "NAVIGATING"

		if self.missions_started:
			for robot in self.active_robots:
				self.active_robots[robot]["navigation"] = self.robot_navigation_status[robot]

	def reload_available_robots_callback(self):
		self.available_list.delete(0,tk.END)
		for robot in self.robot_navigation_status:
			self.available_list.insert(tk.END,robot)

		return

	def assign_missions_and_goals(self,timer):
		if self.missions_started:
			#check if there is any mission queued that can be assigned to a robot
			if len(self.queued_missions) > 0:
				#check if any of the active robots is free
				for robot in self.active_robots:
					if self.active_robots[robot]["status"] == "FREE":
						# assign the mission to the free robot
						self.active_robots[robot]["status"] = "BUSY"
						self.active_robots[robot]["mission"] = self.queued_missions[0]
						del self.queued_missions[0]
			 			break

			#check if the waiting deadline is over for each active robot and the navigation is over
			for robot in self.active_robots:
				if (self.active_robots[robot]["mission"]!= "-" and self.active_robots[robot]["navigation"] == "FREE" and self.active_robots[robot]["wait"]<=rospy.get_time()):
						self.process_new_goal(robot)

				# removing the exploration action tag after a predefined tiem interval so other exploring goals can come in
				if (self.active_robots[robot]["action"]=="EXPLORING" and self.active_robots[robot]["wait"]<=rospy.get_time()):
					self.active_robots[robot]["action"]= "-"


			
			#update the general status of the robot based on the navigation and active mission
			for robot in self.active_robots:
				if (self.active_robots[robot]["navigation"] == "FREE" and self.active_robots[robot]["mission"]=="-"):
					self.active_robots[robot]["status"] = "FREE"
				else:
					self.active_robots[robot]["status"] = "BUSY"

		return

	def process_new_goal(self,robot):
		current_goal = self.active_robots[robot]["goal"]
		current_mission = self.active_robots[robot]["mission"]

		if current_goal == "-":
			new_goal = 0
		else:
			new_goal = current_goal + 1
			if new_goal >= len(self.missions[current_mission]):
				#all goals from this mission completed
				self.active_robots[robot]["goal"] = "-"
				self.active_robots[robot]["mission"] = "-"
				self.active_robots[robot]["wait"] = 0
				self.active_robots[robot]["action"] = "-"
				self.completed_missions.append(current_mission)
				return

		self.active_robots[robot]["goal"] = new_goal
		self.active_robots[robot]["action"] = self.missions[int(current_mission)][int(new_goal)]
		# read the new goal
		goal_description = self.missions[int(current_mission)][int(new_goal)]
		goal_description = goal_description.split(":")
		action = goal_description[0]

		if action == "go":
			#send a goal to the coordinator
			item = goal_description[1]
			if item == 'home':
				location = 'home_'+str(robot)
			else:	
				location = self.item_locations_data[item]
			coordinates = self.location_coordinates_data[location]
			x = coordinates[0]
			y = coordinates[1]
			yaw = coordinates[2]
			print "Coordinates "+str(coordinates)

			location_goal = PoseStamped()
			location_goal.header.stamp = rospy.get_rostime()
			location_goal.header.frame_id = self.locations_frame_id
			location_goal.pose.position.x = coordinates[0] 
			location_goal.pose.position.y = coordinates[1]
			[qx,qy,qz,qw]=tf.transformations.quaternion_from_euler(0,0,coordinates[2])
			location_goal.pose.orientation.x = qx
			location_goal.pose.orientation.y = qy
			location_goal.pose.orientation.z = qz
			location_goal.pose.orientation.w = qw

			if robot == "robot1":
				self.robot1_goal_pub.publish(location_goal)
			elif robot == "robot2":
				self.robot2_goal_pub.publish(location_goal)
			elif robot == "robot3":
				self.robot3_goal_pub.publish(location_goal)
			elif robot == "robot4":
				self.robot4_goal_pub.publish(location_goal)
			elif robot == "robot5":
				self.robot5_goal_pub.publish(location_goal)
			elif robot == "robot6":
				self.robot6_goal_pub.publish(location_goal)
			elif robot == "robot7":
				self.robot7_goal_pub.publish(location_goal)
			elif robot == "robot8":
				self.robot8_goal_pub.publish(location_goal)
			elif robot == "robot9":
				self.robot9_goal_pub.publish(location_goal)

			self.active_robots[robot]["wait"] = rospy.get_time() + 30

		elif action == "pick":
			object = goal_description[1]
			# since there is not actual picking in the sim we use a wait instead
			self.active_robots[robot]["wait"] = rospy.get_time() + 5

		elif action == "drop":
			# now we use a 10 seconds wait instead
			self.active_robots[robot]["wait"] = rospy.get_time() + 10

		else:
			self.active_robots[robot]["wait"] = rospy.get_time() + 10

		return

	def gui_update(self):
		current_time = rospy.get_time()
		self.allow_exploration_value = self.allow_exploration.get()

		#update mission panel
		self.progress_text_label.config(text=str(len(self.completed_missions))+"/"+str(len(self.missions))) 
		self.completed_text_label.config(text="Missions " + str(self.completed_missions))
		self.queued_text_label.config(text="Missions " + str(self.queued_missions))
		if self.all_missions_added or self.missions_started==0:
			self.next_text_label.config(text="-")
		else:
			self.next_text_label.config(text="Mission "+str(self.next_mission)+" in "+str(int(self.corrected_missions_times[self.next_mission]-current_time))+" sec")

		#update robot status panel
		self.status_list.delete(0,tk.END)
		self.mission_list.delete(0,tk.END)
		self.goal_list.delete(0,tk.END)
		self.navigation_list.delete(0,tk.END)
		self.wait_list.delete(0,tk.END)
		self.action_list.delete(0,tk.END)
		if len(self.active_robots) > 0:
			for robot in self.active_robots:
				self.status_list.insert(tk.END,self.active_robots[robot]["status"])
				self.mission_list.insert(tk.END,self.active_robots[robot]["mission"])
				self.navigation_list.insert(tk.END,self.active_robots[robot]["navigation"])
				if self.active_robots[robot]["mission"] == "-":
					self.goal_list.insert(tk.END,str(self.active_robots[robot]["goal"]))
				else:
					self.goal_list.insert(tk.END,str(self.active_robots[robot]["goal"]+1)+"/"+str(len(self.missions[int(self.active_robots[robot]["mission"])])))

				if self.active_robots[robot]["wait"]-current_time >= 0:
					self.wait_list.insert(tk.END,int(self.active_robots[robot]["wait"]-current_time))
				else:
					self.wait_list.insert(tk.END,0)

				self.action_list.insert(tk.END,self.active_robots[robot]["action"])
		else:
			self.active_list.delete(0,tk.END)


		self.gui.update() # non blocking

	def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown() and self.shutdown_node == False:

			# the misison are put in the queue when their starting time arrives
			if (self.missions_started==1 and self.all_missions_added==0):
				if self.corrected_missions_times[self.next_mission] <= rospy.get_time():
					self.queued_missions.append(self.next_mission)
					self.next_mission = self.next_mission + 1
					if self.next_mission >= len(self.corrected_missions_times):
						self.all_missions_added = 1

			#update all the gui fields with the new info
			self.gui_update()
			
			# provide info in topics
			self.active_robots_status_pub.publish(json.dumps(self.active_robots))

			self.mission_status["completed"] = self.completed_missions
			self.mission_status["queued"] = self.queued_missions
			if (self.missions_started==1 and self.all_missions_added==0):
				self.mission_status["seconds_left_next"] = int(self.corrected_missions_times[self.next_mission]-rospy.get_time())
			else:
				self.mission_status["seconds_left_next"] = 	"-"
			self.mission_status_pub.publish(json.dumps(self.mission_status))

			self.exploration_allowed_pub.publish(self.allow_exploration_value) 

			r.sleep()

	
	def robot1_pointclickgoal_callback(self,msg):
		self.robot1_goal_pub.publish(msg)

	def robot2_pointclickgoal_callback(self,msg):
		self.robot2_goal_pub.publish(msg)

	def robot3_pointclickgoal_callback(self,msg):
		self.robot3_goal_pub.publish(msg)

	def robot4_pointclickgoal_callback(self,msg):
		self.robot4_goal_pub.publish(msg)

	def robot5_pointclickgoal_callback(self,msg):
		self.robot5_goal_pub.publish(msg)

	def robot6_pointclickgoal_callback(self,msg):
		self.robot6_goal_pub.publish(msg)

	def robot7_pointclickgoal_callback(self,msg):
		self.robot7_goal_pub.publish(msg)

	def robot8_pointclickgoal_callback(self,msg):
		self.robot8_goal_pub.publish(msg)

	def robot9_pointclickgoal_callback(self,msg):
		self.robot9_goal_pub.publish(msg)



if __name__ == '__main__':
	rospy.init_node('iliad_goal_manager_node', anonymous=True)
	igm = iliad_goal_manager()
