#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose,PoseStamped
import Tkinter as tk
from std_msgs.msg import String, Int16, Empty
import tkMessageBox
import xml.etree.ElementTree as ET
import json
from orunav_msgs.msg import RobotReport, RobotTarget, PoseSteering, Operation, IliadItemArray,IliadItem, ComputeTaskStatus
from orunav_msgs.srv import Trigger
import tf.transformations

class iliad_goal_manager(object):

	def __init__(self):
		#parameters
		self.orders_file = rospy.get_param('~orders_file',"")
		self.parsed_orders_file = rospy.get_param('~parsed_orders_file',"")
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
		self.robot_report_status = {}

		self.start_experiment = False
		

		if self.mode==0: #click and point 
			print "GOAL MANAGER STARTED WITH POINT AND CLICK MODE"
			#subscribers
			rospy.Subscriber("/robot1/goal", PoseStamped, self.robot1_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot2/goal", PoseStamped, self.robot2_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot3/goal", PoseStamped, self.robot3_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot4/goal", PoseStamped, self.robot4_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot5/goal", PoseStamped, self.robot5_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot6/goal", PoseStamped, self.robot6_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot7/goal", PoseStamped, self.robot7_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot8/goal", PoseStamped, self.robot8_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot9/goal", PoseStamped, self.robot9_pointclickgoal_callback,queue_size=1)

			# publishers
			self.robot1_goal_pub = rospy.Publisher("/robot1/robot_target", RobotTarget,queue_size=1)
			self.robot2_goal_pub = rospy.Publisher("/robot2/robot_target", RobotTarget,queue_size=1)
			self.robot3_goal_pub = rospy.Publisher("/robot3/robot_target", RobotTarget,queue_size=1)
			self.robot4_goal_pub = rospy.Publisher("/robot4/robot_target", RobotTarget,queue_size=1)
			self.robot5_goal_pub = rospy.Publisher("/robot5/robot_target", RobotTarget,queue_size=1)
			self.robot6_goal_pub = rospy.Publisher("/robot6/robot_target", RobotTarget,queue_size=1)
			self.robot7_goal_pub = rospy.Publisher("/robot7/robot_target", RobotTarget,queue_size=1)
			self.robot8_goal_pub = rospy.Publisher("/robot8/robot_target", RobotTarget,queue_size=1)
			self.robot9_goal_pub = rospy.Publisher("/robot9/robot_target", RobotTarget,queue_size=1)

			rospy.spin()

		else: # missions mode
			rospy.Subscriber("/robot1/goal", PoseStamped, self.robot1_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot2/goal", PoseStamped, self.robot2_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot3/goal", PoseStamped, self.robot3_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot4/goal", PoseStamped, self.robot4_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot5/goal", PoseStamped, self.robot5_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot6/goal", PoseStamped, self.robot6_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot7/goal", PoseStamped, self.robot7_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot8/goal", PoseStamped, self.robot8_pointclickgoal_callback,queue_size=1)
			rospy.Subscriber("/robot9/goal", PoseStamped, self.robot9_pointclickgoal_callback,queue_size=1)

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

			rospy.Subscriber("/robot1/compute_task/status", ComputeTaskStatus, self.robot1_computetaskstatus_callback,queue_size=1)
			rospy.Subscriber("/robot2/compute_task/status", ComputeTaskStatus, self.robot2_computetaskstatus_callback,queue_size=1)
			rospy.Subscriber("/robot3/compute_task/status", ComputeTaskStatus, self.robot3_computetaskstatus_callback,queue_size=1)
			rospy.Subscriber("/robot4/compute_task/status", ComputeTaskStatus, self.robot4_computetaskstatus_callback,queue_size=1)
			rospy.Subscriber("/robot5/compute_task/status", ComputeTaskStatus, self.robot5_computetaskstatus_callback,queue_size=1)
			rospy.Subscriber("/robot6/compute_task/status", ComputeTaskStatus, self.robot6_computetaskstatus_callback,queue_size=1)
			rospy.Subscriber("/robot7/compute_task/status", ComputeTaskStatus, self.robot7_computetaskstatus_callback,queue_size=1)
			rospy.Subscriber("/robot8/compute_task/status", ComputeTaskStatus, self.robot8_computetaskstatus_callback,queue_size=1)
			rospy.Subscriber("/robot9/compute_task/status", ComputeTaskStatus, self.robot9_computetaskstatus_callback,queue_size=1)

			rospy.Subscriber("start_experiment", Empty, self.start_experiment_callback,queue_size=1)			

			# create topic publishers
			self.active_robots_status_pub = rospy.Publisher('/active_robot_status', String,queue_size=10)
			self.mission_status_pub = rospy.Publisher('/mission_status', String,queue_size=10)
			self.robot1_goal_pub = rospy.Publisher("/robot1/robot_target", RobotTarget,queue_size=1)
			self.robot2_goal_pub = rospy.Publisher("/robot2/robot_target", RobotTarget,queue_size=1)
			self.robot3_goal_pub = rospy.Publisher("/robot3/robot_target", RobotTarget,queue_size=1)
			self.robot4_goal_pub = rospy.Publisher("/robot4/robot_target", RobotTarget,queue_size=1)
			self.robot5_goal_pub = rospy.Publisher("/robot5/robot_target", RobotTarget,queue_size=1)
			self.robot6_goal_pub = rospy.Publisher("/robot6/robot_target", RobotTarget,queue_size=1)
			self.robot7_goal_pub = rospy.Publisher("/robot7/robot_target", RobotTarget,queue_size=1)
			self.robot8_goal_pub = rospy.Publisher("/robot8/robot_target", RobotTarget,queue_size=1)
			self.robot9_goal_pub = rospy.Publisher("/robot9/robot_target", RobotTarget,queue_size=1)
			self.exploration_allowed_pub = rospy.Publisher("exploration_allowed",Int16,queue_size=1)
			
			#services
			#rospy.wait_for_service('/coordinator/abort')
			self.abort_goal_service_client = rospy.ServiceProxy('/coordinator/abort',Trigger)


			# create timers
			self.assign_missions_and_goals_timer = rospy.Timer(rospy.Duration(2),self.assign_missions_and_goals)
			
			#parse input files
			# read orders file
			if self.orders_file != "":
				print "READING ORDERS FILE"
				self.orders_data = self.parse_orders_file()
			elif self.parsed_orders_file != "":
				print "READING PARSED FILE"
				with open(self.parsed_orders_file,"r") as file:
					self.orders_data = json.load(file)
			else:
				print "NO MISSIONS BEING PROVIDED - CLOSING NODE"
				return

			self.number_of_orders = len(self.orders_data["orders"])

			self.parse_orders_times_file()

			if self.items_locations_file != "" and self.locations_coordinates_file != "":
				self.parse_item_locations_file()
				self.parse_location_coordinates_file()
			else:
				print "NO ITEM OR COORDINATES LOCATIONS PROVIDED - CLOSING NODE"
				return
			
			#start gui variables		
			self.start_gui()
			
			# ini the main loop
			self.run()

	def parse_orders_file(self):
		orders_data = {"orders":[]}
		mission= []		
		orders_xml = ET.parse(self.orders_file).getroot()

		for o_num in range(0,len(orders_xml)):
			mixed_pallets_num = 0
			full_pallets_num = 0
			order = {}
			order["id"] = orders_xml[o_num].attrib["orderId"] #o_num
			order["starting_time"] = 0
			order["mission"] = []
			for pallet_type in range(0,len(orders_xml[o_num])):
				if orders_xml[o_num][pallet_type].tag == "FullPallets" and len(orders_xml[o_num][pallet_type]) > 0:
					for pallet in range(0,len(orders_xml[o_num][pallet_type])):
						full_pallets_num = full_pallets_num + 1
						item_name = orders_xml[o_num][pallet_type][pallet][0][0].attrib["name"]
						goal = {}
						goal["location"] = "fullpallet_"+item_name
						goal["operation"] = "LOAD_DETECT"
						order["mission"].append(goal)

						goal = {}
						goal["location"] = "shipping_area"
						goal["operation"] = "NO_OPERATION"
						order["mission"].append(goal)

						goal = {}
						goal["location"] = "shipping_area"
						goal["operation"] = "UNLOAD"
						order["mission"].append(goal)


				if orders_xml[o_num][pallet_type].tag == "MixedPallets" and len(orders_xml[o_num][pallet_type]) > 0:
					for pallet in range(0,len(orders_xml[o_num][pallet_type])):
						mixed_pallets_num = mixed_pallets_num + 1
						goal = {}
						goal["location"] = "empty_pallet"
						goal["operation"] = "LOAD_DETECT"
						order["mission"].append(goal)


						goal = {}
						goal["items"] = []

						previous_item_name = orders_xml[o_num][pallet_type][pallet][0][0].attrib["name"]
						for item_number in range(0,len(orders_xml[o_num][pallet_type][pallet][0])):
							item_name = orders_xml[o_num][pallet_type][pallet][0][item_number].attrib["name"]
							if item_name == previous_item_name:
								item = {}
								item["name"] = item_name
								item["x"] = orders_xml[o_num][pallet_type][pallet][0][item_number][0].text
								item["y"] = orders_xml[o_num][pallet_type][pallet][0][item_number][1].text
								item["z"] = orders_xml[o_num][pallet_type][pallet][0][item_number][2].text
								item["rotation"] = orders_xml[o_num][pallet_type][pallet][0][item_number][3].text
								goal["items"].append(item)
								previous_item_name = item_name

							else:
								goal["location"] = previous_item_name
								goal["operation"] = "PICK_ITEMS"
								order["mission"].append(goal)
								
								goal = {}
								goal["items"] = []
								
								item = {}
								item["name"] = item_name
								item["x"] = orders_xml[o_num][pallet_type][pallet][0][item_number][0].text
								item["y"] = orders_xml[o_num][pallet_type][pallet][0][item_number][1].text
								item["z"] = orders_xml[o_num][pallet_type][pallet][0][item_number][2].text
								item["rotation"] = orders_xml[o_num][pallet_type][pallet][0][item_number][3].text
								goal["items"].append(item)

								previous_item_name = item_name

						goal["location"] = item_name
						goal["operation"] = "PICK_ITEMS"
						order["mission"].append(goal)

						goal = {}
						goal["location"] = "shipping_area"
						goal["operation"] = "NO_OPERATION"
						order["mission"].append(goal)

						goal = {}
						goal["location"] = "shipping_area"
						goal["operation"] = "UNLOAD"
						order["mission"].append(goal)



			goal = {}
			goal["location"] = "home"
			goal["operation"] = "NO_OPERATION"
			order["mission"].append(goal)

			order["mixed_pallets"] = mixed_pallets_num
			order["full_pallets"] = full_pallets_num
			orders_data["orders"].append(order)

			with open(self.orders_file[:-4]+"_parsed.json", 'w') as pfile:
				json.dump(orders_data, pfile, indent=4, sort_keys=True)
		return orders_data

	def parse_orders_times_file(self):
		if self.orders_times_file != "":
			with open(self.orders_times_file,"r") as file:
				orders_times_data = json.load(file)

			self.missions_times = []
			for o in range(0,self.number_of_orders):
				try:
					time_ = orders_times_data[self.orders_data["orders"][o]["id"]]
					print "Updating time for order "+ self.orders_data["orders"][o]["id"]
					self.orders_data["orders"][o]["starting_time"] = time_
					self.missions_times.append(time_)
				except:
					print "No time given for order "+ self.orders_data["orders"][o]["id"]+ " . Keeping it as 0."
					self.missions_times.append(0)
		else:
			print "NO TIMES FILE PROVIDED"
			self.missions_times = []
			for o in range(0,self.number_of_orders):
				self.missions_times.append(0)

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

		self.mission_mode_frame = tk.LabelFrame(self.gui,text="ORDERS MODE",font=("Arial Bold", 12))
		self.mission_mode_frame.grid(row=0,column=1,padx=10, pady=10,sticky="w")
		
		self.files_frame = tk.LabelFrame(self.mission_mode_frame,text="Files Loaded",font=("Arial Bold", 12))
		self.files_frame.grid(row=0,column=0,padx=10, pady=10,sticky="w")

		self.missions_frame = tk.LabelFrame(self.mission_mode_frame,text="Mission Status",font=("Arial Bold", 12))
		self.missions_frame.grid(row=1,column=0,padx=10, pady=10,sticky="w")

		self.robots_frame = tk.LabelFrame(self.mission_mode_frame,text="Robot Status",font=("Arial Bold", 12))
		self.robots_frame.grid(row=2,column=0,padx=10, pady=10,sticky="w")

		#files frame
		orders_label = tk.Label(self.files_frame, text = "Orders: ",font=("Arial Bold", 10))
		parsed_orders_label = tk.Label(self.files_frame, text = "Parsed orders: ",font=("Arial Bold", 10))
		orders_times_label = tk.Label(self.files_frame, text = "Orders times: ",font=("Arial Bold", 10)) 
		items_locations_label = tk.Label(self.files_frame, text = "Items locations: ",font=("Arial Bold", 10))
		locations_coordinates_label = tk.Label(self.files_frame, text = "Locations coordinates: ",font=("Arial Bold", 10))
		orders_label.grid(               row=0,column=0,pady=3,padx=5,sticky="e")
		parsed_orders_label.grid(        row=1,column=0,pady=3,padx=5,sticky="e")
		orders_times_label.grid(         row=2,column=0,pady=3,padx=5,sticky="e")
		items_locations_label.grid(      row=3,column=0,pady=3,padx=5,sticky="e")
		locations_coordinates_label.grid(row=4,column=0,pady=3,padx=5,sticky="e")

		orders_text = tk.Label(self.files_frame, text = self.orders_file,font=("Arial Bold", 10),bg="white",width=100)
		parsed_orders_text = tk.Label(self.files_frame, text = self.parsed_orders_file,font=("Arial Bold", 10),bg="white",width=100)
		orders_times_text = tk.Label(self.files_frame, text = self.orders_times_file,font=("Arial Bold", 10),bg="white",width=100)
		items_locations_text = tk.Label(self.files_frame,       text = self.items_locations_file,font=("Arial Bold", 10),bg="white",width=100)
		locations_coordinates_text = tk.Label(self.files_frame, text = self.locations_coordinates_file,font=("Arial Bold", 10),bg="white",width=100)
		orders_text.grid(               row=0,column=1,sticky="w")
		parsed_orders_text.grid(        row=1,column=1,sticky="w")
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

		start_missions_button = tk.Button(self.missions_frame, text ="Start\norders", command = self.start_missions_callback,height=5, width=5,background="green",activebackground="green",font=("Arial Bold", 10))
		start_missions_button.grid(row=1,rowspan=4,column = 2,padx=5)

		progress_label = tk.Label(self.missions_frame, text = "Progress: ",font=("Arial Bold", 10),justify="right")
		completed_label = tk.Label(self.missions_frame, text = "Completed: ",font=("Arial Bold", 10),justify="right")
		queued_label = tk.Label(self.missions_frame, text = "Queued: ",font=("Arial Bold", 10),justify="right")
		next_label =      tk.Label(self.missions_frame, text = "Next: ",font=("Arial Bold", 10),justify="right")
		progress_label.grid( row=1,column=3,sticky="e",pady=3,padx=5)
		completed_label.grid(row=2,column=3,sticky="e",pady=3,padx=5)
		queued_label.grid(   row=3,column=3,sticky="e",pady=3,padx=5)
		next_label.grid(     row=4,column=3,sticky="e",pady=3,padx=5)

		self.progress_text_label = tk.Label(self.missions_frame,font=("Arial Bold", 10),bg="white",width = 45)
		self.completed_text_label = tk.Label(self.missions_frame,font=("Arial Bold", 10),bg="white",width = 45)
		self.queued_text_label = tk.Label(self.missions_frame,font=("Arial Bold", 10),bg="white",width = 45)
		self.next_text_label =      tk.Label(self.missions_frame,font=("Arial Bold", 10),bg="white",width = 45)
		self.progress_text_label.grid( row=1,column=4,sticky="w")
		self.completed_text_label.grid(row=2,column=4,sticky="w")
		self.queued_text_label.grid(   row=3,column=4,sticky="w")
		self.next_text_label.grid(     row=4,column=4,sticky="w")

		abort_missions_button = tk.Button(self.missions_frame, text ="Abort\nall\norders", command = self.abort_missions_callback,height=5, width=5,background="red",activebackground="red",font=("Arial Bold", 10))
		abort_missions_button.grid(row=1,rowspan=4,column = 5,padx=5)

		# robot status panel
		active_label = tk.Label(self.robots_frame, text = "Active",font=("Arial Bold", 10))
		status_label = tk.Label(self.robots_frame, text = "Status",font=("Arial Bold", 10))
		mission_label = tk.Label(self.robots_frame, text = "Mission",font=("Arial Bold", 10))
		report_label = tk.Label(self.robots_frame, text = "Report",font=("Arial Bold", 10))
		wait_label = tk.Label(self.robots_frame, text = "Wait",font=("Arial Bold", 10))
		goal_label = tk.Label(self.robots_frame, text = "Goal",font=("Arial Bold", 10))
		location_label = tk.Label(self.robots_frame, text = "Item/Location",font=("Arial Bold", 10))
		operation_label = tk.Label(self.robots_frame, text = "Operation",font=("Arial Bold", 10))
		active_label.grid(row=0,column=0,pady=5,padx=5)
		status_label.grid(row=0,column=1,pady=5,padx=5)
		mission_label.grid(row=0,column=2,pady=5,padx=5)
		goal_label.grid(row=0,column=3,pady=5,padx=5)
		wait_label.grid(row=0,column=4,pady=5,padx=5)
		report_label.grid(row=0,column=5,pady=5,padx=5)
		location_label.grid(row=0,column=6,pady=5,padx=5)
		operation_label.grid(row=0,column=7,pady=5,padx=5)


		self.active_list = tk.Listbox(self.robots_frame,width=15,font=("Arial Bold", 10),height=4,justify="center")
		self.status_list = tk.Listbox(self.robots_frame,width=6,font=("Arial Bold", 10), height=4,justify="center")
		self.mission_list = tk.Listbox(self.robots_frame,width=7,font=("Arial Bold", 10),height=4,justify="center")
		self.report_list = tk.Listbox(self.robots_frame,width=30,font=("Arial Bold", 10),height=4,justify="center")
		self.wait_list = tk.Listbox(self.robots_frame,width=5,font=("Arial Bold", 10),height=4,justify="center")
		self.goal_list = tk.Listbox(self.robots_frame,width=10,font=("Arial Bold", 10),height=4,justify="center")
		self.location_list = tk.Listbox(self.robots_frame,width=30,font=("Arial Bold", 10),height=4,justify="center")
		self.operation_list = tk.Listbox(self.robots_frame,width=15,font=("Arial Bold", 10),height=4,justify="center")
		
		self.active_list.grid(row=1,column=0,padx=5,pady=5)
		self.status_list.grid(row=1,column=1,padx=5,pady=5)
		self.mission_list.grid(row=1,column=2,padx=5,pady=5)
		self.goal_list.grid(row=1,column=3,padx=5,pady=5)
		self.wait_list.grid(row=1,column=4,padx=5,pady=5)
		self.report_list.grid(row=1,column=5,padx=5,pady=5)
		self.location_list.grid(row=1,column=6,padx=5,pady=5)
		self.operation_list.grid(row=1,column=7,padx=5,pady=5)



		#skip_goal_button = tk.Button(self.robots_frame, text ="Skip\ncurrent\ngoal", command = self.skip_goal_callback, width=5,background="orange",activebackground="orange",font=("Arial Bold", 10))
		skip_mission_button = tk.Button(self.robots_frame, text ="Cancel\ncurrent\norder", command = self.skip_mission_callback, width=5,background="red",activebackground="red",font=("Arial Bold", 10))
		#skip_goal_button.grid(row=1,column = 8,padx=5)
		skip_mission_button.grid(row=1,column = 9,padx=5)

		#self.gui.mainloop() #blocking

	def start_experiment_callback(self, msg):
		self.start_experiment = True

	def start_missions_callback(self):
		print "starting the missions"
		if self.missions_started == 0:
			#update the active robots list from the selection
			robot_names = self.available_list.get(0,tk.END)
			robot_selection =  self.available_list.curselection()

			# added for automation
			robot_names = ('robot1',)
			robot_selection = (0,)

			if len(robot_selection) > 0:
				for i in range(0,len(robot_selection)):
					self.active_list.insert(tk.END,robot_names[robot_selection[i]])
					self.active_robots[robot_names[robot_selection[i]]] = {"status":"-","mission":"-","goal":"-","wait":0,"report":"-","location":"-","operation":"-","computing_task":0}

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

		for robot in self.active_robots:
			if self.robot_report_status[robot] != "WAITING_FOR_TASK":
				self.abort_goal(robot)

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
			# 	if self.active_robots[robot]["mission"] != "-":
			# 		self.active_robots[robot]["goal"] = "-"
			# 		self.active_robots[robot]["mission"] = "-"
			# 		self.active_robots[robot]["wait"] = 0
			# 		self.active_robots[robot]["location"] = "-"
			# 		self.active_robots[robot]["operation"] = "-"

		return

	def finish_missions(self):
		print "all missions finished"
		self.available_list_updated = 0
		self.missions_started = 0
		self.available_robots = {}
		self.active_robots = {}
		self.next_mission = 0
		self.queued_missions = []
		self.all_missions_added = 0
		self.completed_missions = []

	def skip_goal_callback(self):
		if len(self.active_list.curselection()) > 0:
			robot_names = self.active_list.get(0,tk.END)
			robot_selected  = robot_names[self.active_list.curselection()[0]]
			print "skippin goal in robot: ", robot_selected

			if self.active_robots[robot_selected]["mission"] != "-":
				if self.robot_report_status[robot_selected] != "WAITING_FOR_TASK":
					self.abort_goal(robot_selected)
				self.process_new_goal(robot_selected)
		else:
			tkMessageBox.showwarning("Warning","Select 1 robot\nfrom the active list")

		return

	def skip_mission_callback(self):
		if len(self.active_list.curselection()) > 0:
			robot_names = self.active_list.get(0,tk.END)
			robot_selected  = robot_names[self.active_list.curselection()[0]]
			print "Cancel order in: ", robot_selected
			if self.robot_report_status[robot_selected] != "WAITING_FOR_TASK":
				self.abort_goal(robot_selected)

			
			if self.active_robots[robot_selected]["mission"] != "-":
				self.queued_missions.append(self.active_robots[robot_selected]["mission"] )
				self.active_robots[robot_selected]["goal"] = "-"
				self.active_robots[robot_selected]["mission"] = "-"
				self.active_robots[robot_selected]["wait"] = 0
				self.active_robots[robot_selected]["location"] = "-"
				self.active_robots[robot_selected]["operation"] = "-"

			#remove the robot associated to the cancelled orders so it cannot be assigned more orders
			self.active_robots.pop(robot_selected,None)

			# update the active robots list
			self.active_list.delete(0,tk.END)
			for robot in self.active_robots:
				self.active_list.insert(tk.END,robot)

		else:
			tkMessageBox.showwarning("Warning","Select 1 robot\nfrom the active list")

		return

	def abort_goal(self,robot):
		robot_num = robot[-1]
		#call the service to abort task
		self.abort_goal_service_client(int(robot_num))

		# this is removed because the aborting does not workign in many occasion and it block the system
		#what for the robot navigation status to be free/idle
		#while self.robot_report_status[robot] != "WAITING_FOR_TASK":
		#	print "...aborting goal in robot",robot
		#	rospy.sleep(0.2)
		#print "goal aborted"

	def exploration_goal_callback(self,exploration_goal_msg):
		print "Exploration goal request received"
		already_exploring = 0
		for robot in self.active_robots:
			if self.active_robots[robot]["operation"] == "EXPLORING":
				already_exploring = 1
				print "Robot already exploring"
				break
		
		if self.allow_exploration_value and already_exploring==0:
			for robot in self.active_robots:
				if self.active_robots[robot]["status"] == "FREE":
					#send an exploration to the free robot and set variable as exploration ongoing
					self.active_robots[robot]["operation"] = "EXPLORING"
					self.active_robots[robot]["wait"] = rospy.get_time() + 150 #after 120 sec the exploring tag is removed

					robotgoal_msg = RobotTarget()
					robotgoal_msg.robot_id = int(robot[-1])
					robotgoal_msg.goal.pose = exploration_goal_msg.pose
					robotgoal_msg.goal_op.operation = 1

					if robot == "robot1":
						self.robot1_goal_pub.publish(robotgoal_msg)
					elif robot == "robot2":
						self.robot2_goal_pub.publish(robotgoal_msg)
					elif robot == "robot3":
						self.robot3_goal_pub.publish(robotgoal_msg)
					elif robot == "robot4":
						self.robot4_goal_pub.publish(robotgoal_msg)
					elif robot == "robot5":
						self.robot5_goal_pub.publish(robotgoal_msg)
					elif robot == "robot6":
						self.robot6_goal_pub.publish(robotgoal_msg)
					elif robot == "robot7":
						self.robot7_goal_pub.publish(robotgoal_msg)
					elif robot == "robot8":
						self.robot8_goal_pub.publish(robotgoal_msg)
					elif robot == "robot9":
						self.robot9_goal_pub.publish(robotgoal_msg)

					self.exploration_ongoing = True
					break
		else:
			print "Exploration not allowed"
		return

	def reload_available_robots_callback(self):
		self.available_list.delete(0,tk.END)
		for robot in self.robot_report_status:
			self.available_list.insert(tk.END,robot)

		return

################################################
##############################     MAIN FUNCTION
################################################ 
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
				if (self.active_robots[robot]["mission"]!= "-" and self.active_robots[robot]["report"] == "WAITING_FOR_TASK" and self.active_robots[robot]["wait"]<=rospy.get_time()):
						self.process_new_goal(robot)

				# removing the exploration action tag after a predefined tiem interval so other exploring goals can come in
				if (self.active_robots[robot]["operation"]=="EXPLORING" and self.active_robots[robot]["wait"]<=rospy.get_time()):
					self.active_robots[robot]["operation"]= "-"


			
			#update the general status of the robot based on the navigation and the waiting time
			for robot in self.active_robots:
				if (self.active_robots[robot]["report"] != "WAITING_FOR_TASK" or self.active_robots[robot]["wait"]>=rospy.get_time()) or self.active_robots[robot]["computing_task"] == 1:
					self.active_robots[robot]["status"] = "BUSY"
				else:
					self.active_robots[robot]["status"] = "FREE"

		return

###################################################
###################################################
###################################################

	def process_new_goal(self,robot):
		current_goal = self.active_robots[robot]["goal"]
		current_mission = self.active_robots[robot]["mission"] 

		if current_goal == "-":
			new_goal = 0
		else:
			new_goal = current_goal + 1
			if new_goal >= len(self.orders_data["orders"][int(current_mission)]["mission"]):
				#all goals from this mission completed
				self.active_robots[robot]["goal"] = "-"
				self.active_robots[robot]["mission"] = "-"
				self.active_robots[robot]["wait"] = 0
				self.active_robots[robot]["location"] = "-"
				self.active_robots[robot]["operation"] = "-"
				self.completed_missions.append(current_mission)
				return
		self.active_robots[robot]["goal"] = int(new_goal)


		# define the robottarget message to send to the coordinator
		robotgoal_msg = RobotTarget()
		robotgoal_msg.robot_id = int(robot[-1])

		# calculate location goal coordinates
		item = self.orders_data["orders"][int(current_mission)]["mission"][int(new_goal)]["location"]
		if item == 'home':
			item = 'home_'+str(robot)
			location = self.item_locations_data[item]
		else:	
			location = self.item_locations_data[item]
		coordinates = self.location_coordinates_data[location]

		goal_msg = PoseSteering()
		goal_msg.pose = Pose()
		goal_msg.pose.position.x = coordinates[0]
		goal_msg.pose.position.y = coordinates[1]
		[qx,qy,qz,qw]=tf.transformations.quaternion_from_euler(0,0,coordinates[2])
		goal_msg.pose.orientation.x = qx
		goal_msg.pose.orientation.y = qy
		goal_msg.pose.orientation.z = qz
		goal_msg.pose.orientation.w = qw

		robotgoal_msg.goal = goal_msg
		self.active_robots[robot]["location"] = item+" / "+location

		# define operation to do at goal or at start
		operation_msg = Operation()
		operation = self.orders_data["orders"][int(current_mission)]["mission"][int(new_goal)]["operation"]

		if operation == "NO_OPERATION":
			robotgoal_msg.start_op.operation = 1
			operation_msg.operation = 1
			robotgoal_msg.goal_op = operation_msg

		if operation == "UNLOAD":
			operation_msg.operation = 2
			robotgoal_msg.start_op = operation_msg
			robotgoal_msg.current_load.status = 1
			robotgoal_msg.goal_load.status = 0
			robotgoal_msg.goal_op.operation = 1

		if operation == "LOAD":
			robotgoal_msg.start_op.operation = 1
			operation_msg.operation = 3
			robotgoal_msg.goal_op = operation_msg

		if operation == "LOAD_DETECT":
			robotgoal_msg.start_op.operation = 1
			operation_msg.operation = 4
			robotgoal_msg.goal_op = operation_msg

		if operation == "ACTIVATE_SUPPORT_LEGS":
			robotgoal_msg.start_op.operation = 1
			operation_msg.operation = 5
			robotgoal_msg.goal_op = operation_msg

		if operation == "LOAD_DETECT_ACTIVE":
			robotgoal_msg.start_op.operation = 1
			operation_msg.operation = 6
			robotgoal_msg.goal_op = operation_msg

		if operation == "PICK_ITEMS":
			robotgoal_msg.start_op.operation = 1
			operation_msg.operation = 7

			#check which items to pick
			itemlist_msg = IliadItemArray()
			for i in range(0,len(self.orders_data["orders"][int(current_mission)]["mission"][int(new_goal)]["items"])):
				item_msg = IliadItem()
				item_msg.name = self.orders_data["orders"][int(current_mission)]["mission"][int(new_goal)]["items"][i]["name"]
				item_msg.position.x = float(self.orders_data["orders"][int(current_mission)]["mission"][int(new_goal)]["items"][i]["x"])
				item_msg.position.y = float(self.orders_data["orders"][int(current_mission)]["mission"][int(new_goal)]["items"][i]["y"])
				item_msg.position.z = float(self.orders_data["orders"][int(current_mission)]["mission"][int(new_goal)]["items"][i]["z"])
				rotation_type = self.orders_data["orders"][int(current_mission)]["mission"][int(new_goal)]["items"][i]["rotation"]
				if rotation_type == "NONE":
					item_msg.rotation_type = 0
				if rotation_type == "X":
					item_msg.rotation_type = 1
				if rotation_type == "Y":
					item_msg.rotation_type = 2
				if rotation_type == "Z":
					item_msg.rotation_type = 3
				if rotation_type == "XZ":
					item_msg.rotation_type = 4
				if rotation_type == "ZX":
					item_msg.rotation_type = 5

				itemlist_msg.items.append(item_msg)
			operation_msg.itemlist = itemlist_msg

			robotgoal_msg.goal_op = operation_msg

		if operation == "UNWRAP_PALLET":
			operation_msg.operation = 8
			robotgoal_msg.goal_op = operation_msg
		
		self.active_robots[robot]["operation"] = operation

		if robot == "robot1":
			self.robot1_goal_pub.publish(robotgoal_msg)
		elif robot == "robot2":
			self.robot2_goal_pub.publish(robotgoal_msg)
		elif robot == "robot3":
			self.robot3_goal_pub.publish(robotgoal_msg)
		elif robot == "robot4":
			self.robot4_goal_pub.publish(robotgoal_msg)
		elif robot == "robot5":
			self.robot5_goal_pub.publish(robotgoal_msg)
		elif robot == "robot6":
			self.robot6_goal_pub.publish(robotgoal_msg)
		elif robot == "robot7":
			self.robot7_goal_pub.publish(robotgoal_msg)
		elif robot == "robot8":
			self.robot8_goal_pub.publish(robotgoal_msg)
		elif robot == "robot9":
			self.robot9_goal_pub.publish(robotgoal_msg)

		self.active_robots[robot]["computing_task"] = 1

		# add waiting time for specific operations 
		if operation == "PICK_ITEMS":
			self.active_robots[robot]["wait"] = rospy.get_time() + 5
		else:
			self.active_robots[robot]["wait"] = rospy.get_time() + 5

		return

	def gui_update(self):
		current_time = rospy.get_time()
		self.allow_exploration_value = self.allow_exploration.get()

		#update mission panel
		self.progress_text_label.config(text=str(len(self.completed_missions))+"/"+str(self.number_of_orders)) 
		self.completed_text_label.config(text="Orders " + str(self.completed_missions))
		self.queued_text_label.config(text="Orders " + str(self.queued_missions))
		if self.all_missions_added or self.missions_started==0:
			self.next_text_label.config(text="-")
		else:
			self.next_text_label.config(text="Mission "+str(self.next_mission)+" in "+str(int(self.corrected_missions_times[self.next_mission]-current_time))+" sec")

		#update robot status panel
		self.status_list.delete(0,tk.END)
		self.mission_list.delete(0,tk.END)
		self.goal_list.delete(0,tk.END)
		self.report_list.delete(0,tk.END)
		self.wait_list.delete(0,tk.END)
		self.location_list.delete(0,tk.END)
		self.operation_list.delete(0,tk.END)
		if len(self.active_robots) > 0:
			for robot in self.active_robots:
				self.status_list.insert(tk.END,self.active_robots[robot]["status"])
				self.mission_list.insert(tk.END,self.active_robots[robot]["mission"])
				self.report_list.insert(tk.END,self.active_robots[robot]["report"])
				if self.active_robots[robot]["mission"] == "-":
					self.goal_list.insert(tk.END,str(self.active_robots[robot]["goal"]))
				else:
					self.goal_list.insert(tk.END,str(self.active_robots[robot]["goal"]+1)+"/"+str(len(self.orders_data["orders"][int(self.active_robots[robot]["mission"])]["mission"])))

				if self.active_robots[robot]["wait"]-current_time >= 0:
					self.wait_list.insert(tk.END,int(self.active_robots[robot]["wait"]-current_time))
				else:
					self.wait_list.insert(tk.END,0)

				self.location_list.insert(tk.END,self.active_robots[robot]["location"])
				self.operation_list.insert(tk.END,self.active_robots[robot]["operation"])
		else:
			self.active_list.delete(0,tk.END)


		self.gui.update() # non blocking

	def run(self):
		r = rospy.Rate(10)
		while not rospy.is_shutdown() and self.shutdown_node == False:

			if self.start_experiment == True:
				self.start_missions_callback()
				self.start_experiment = False 

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

			if self.all_missions_added == 1 and len(self.mission_status["completed"])==self.number_of_orders and self.allow_exploration_value == 0:
				self.finish_missions()

			r.sleep()

	def robot1_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_report_status["robot1"] = "WAITING_FOR_TASK"
		if  msg.status == 2:
			self.robot_report_status["robot1"] = "PERFORMING_START_OPERATION"
		if  msg.status == 3:
			self.robot_report_status["robot1"] = "DRIVING"
		if  msg.status == 4:
			self.robot_report_status["robot1"] = "PERFORMING_GOAL_OPERATION"
		if  msg.status == 5:
			self.robot_report_status["robot1"] = "TASK_FAILED"

		if self.missions_started:
			if msg.status == 2 or msg.status == 3 or msg.status==4:
				self.active_robots["robot1"]["computing_task"] = 0

			if self.active_robots["robot1"]["computing_task"] == 0:
				self.active_robots["robot1"]["report"] = self.robot_report_status["robot1"]

	def robot2_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_report_status["robot2"] = "WAITING_FOR_TASK"
		if  msg.status == 2:
			self.robot_report_status["robot2"] = "PERFORMING_START_OPERATION"
		if  msg.status == 3:
			self.robot_report_status["robot2"] = "DRIVING"
		if  msg.status == 4:
			self.robot_report_status["robot2"] = "PERFORMING_GOAL_OPERATION"
		if  msg.status == 5:
			self.robot_report_status["robot2"] = "TASK_FAILED"

		if self.missions_started:
			if msg.status == 2 or msg.status == 3 or msg.status==4:
				self.active_robots["robot2"]["computing_task"] = 0

			if self.active_robots["robot2"]["computing_task"] == 0:
				self.active_robots["robot2"]["report"] = self.robot_report_status["robot2"]

	def robot3_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_report_status["robot3"] = "WAITING_FOR_TASK"
		if  msg.status == 2:
			self.robot_report_status["robot3"] = "PERFORMING_START_OPERATION"
		if  msg.status == 3:
			self.robot_report_status["robot3"] = "DRIVING"
		if  msg.status == 4:
			self.robot_report_status["robot3"] = "PERFORMING_GOAL_OPERATION"
		if  msg.status == 5:
			self.robot_report_status["robot3"] = "TASK_FAILED"

		if self.missions_started:
			if msg.status == 2 or msg.status == 3 or msg.status==4:
				self.active_robots["robot3"]["computing_task"] = 0

			if self.active_robots["robot3"]["computing_task"] == 0:
				self.active_robots["robot3"]["report"] = self.robot_report_status["robot3"]

	def robot4_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_report_status["robot4"] = "WAITING_FOR_TASK"
		if  msg.status == 2:
			self.robot_report_status["robot4"] = "PERFORMING_START_OPERATION"
		if  msg.status == 3:
			self.robot_report_status["robot4"] = "DRIVING"
		if  msg.status == 4:
			self.robot_report_status["robot4"] = "PERFORMING_GOAL_OPERATION"
		if  msg.status == 5:
			self.robot_report_status["robot4"] = "TASK_FAILED"

		if self.missions_started:
			if msg.status == 2 or msg.status == 3 or msg.status==4:
				self.active_robots["robot4"]["computing_task"] = 0

			if self.active_robots["robot4"]["computing_task"] == 0:
				self.active_robots["robot4"]["report"] = self.robot_report_status["robot4"]

	def robot5_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_report_status["robot5"] = "WAITING_FOR_TASK"
		if  msg.status == 2:
			self.robot_report_status["robot5"] = "PERFORMING_START_OPERATION"
		if  msg.status == 3:
			self.robot_report_status["robot5"] = "DRIVING"
		if  msg.status == 4:
			self.robot_report_status["robot5"] = "PERFORMING_GOAL_OPERATION"
		if  msg.status == 5:
			self.robot_report_status["robot5"] = "TASK_FAILED"

		if self.missions_started:
			if msg.status == 2 or msg.status == 3 or msg.status==4:
				self.active_robots["robot5"]["computing_task"] = 0

			if self.active_robots["robot5"]["computing_task"] == 0:
				self.active_robots["robot5"]["report"] = self.robot_report_status["robot5"]

	def robot6_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_report_status["robot6"] = "WAITING_FOR_TASK"
		if  msg.status == 2:
			self.robot_report_status["robot6"] = "PERFORMING_GOAL_OPERATION"
		if  msg.status == 3:
			self.robot_report_status["robot6"] = "DRIVING"
		if  msg.status == 4:
			self.robot_report_status["robot6"] = "PERFORMING_START_OPERATION"
		if  msg.status == 5:
			self.robot_report_status["robot6"] = "TASK_FAILED"

		if self.missions_started:
			if msg.status == 2 or msg.status == 3 or msg.status==4:
				self.active_robots["robot6"]["computing_task"] = 0

			if self.active_robots["robot6"]["computing_task"] == 0:
				self.active_robots["robot6"]["report"] = self.robot_report_status["robot6"]

	def robot7_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_report_status["robot7"] = "WAITING_FOR_TASK"
		if  msg.status == 2:
			self.robot_report_status["robot7"] = "PERFORMING_START_OPERATION"
		if  msg.status == 3:
			self.robot_report_status["robot7"] = "DRIVING"
		if  msg.status == 4:
			self.robot_report_status["robot7"] = "PERFORMING_GOAL_OPERATION"
		if  msg.status == 5:
			self.robot_report_status["robot7"] = "TASK_FAILED"

		if self.missions_started:
			if msg.status == 2 or msg.status == 3 or msg.status==4:
				self.active_robots["robot7"]["computing_task"] = 0

			if self.active_robots["robot7"]["computing_task"] == 0:
				self.active_robots["robot7"]["report"] = self.robot_report_status["robot7"]

	def robot8_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_report_status["robot8"] = "WAITING_FOR_TASK"
		if  msg.status == 2:
			self.robot_report_status["robot8"] = "PERFORMING_START_OPERATION"
		if  msg.status == 3:
			self.robot_report_status["robot8"] = "DRIVING"
		if  msg.status == 4:
			self.robot_report_status["robot8"] = "PERFORMING_GOAL_OPERATION"
		if  msg.status == 5:
			self.robot_report_status["robot8"] = "TASK_FAILED"

		if self.missions_started:
			if msg.status == 2 or msg.status == 3 or msg.status==4:
				self.active_robots["robot8"]["computing_task"] = 0

			if self.active_robots["robot8"]["computing_task"] == 0:
				self.active_robots["robot8"]["report"] = self.robot_report_status["robot8"]

	def robot9_status_callback(self,msg):
		if  msg.status == 1:
			self.robot_report_status["robot9"] = "WAITING_FOR_TASK"
		if  msg.status == 2:
			self.robot_report_status["robot9"] = "PERFORMING_START_OPERATION"
		if  msg.status == 3:
			self.robot_report_status["robot9"] = "DRIVING"
		if  msg.status == 4:
			self.robot_report_status["robot9"] = "PERFORMING_GOAL_OPERATION"
		if  msg.status == 5:
			self.robot_report_status["robot9"] = "TASK_FAILED"

		if self.missions_started:
			if msg.status == 2 or msg.status == 3 or msg.status==4:
				self.active_robots["robot9"]["computing_task"] = 0

			if self.active_robots["robot9"]["computing_task"] == 0:
				self.active_robots["robot9"]["report"] = self.robot_report_status["robot9"]

	def robot1_computetaskstatus_callback(self,msg):
		if msg.status == 0:
			message = "COMPUTE_TASK_START"
		if msg.status == 1:
			message = "COMPUTE_TASK_SUCCESS"
		if msg.status == 2:	
			message = "INVALID_TARGET"
		if msg.status == 3:
			message = "INVALID_MAP"
		if msg.status == 4:
			message = "INVALID_START"
		if msg.status == 5:	
			message = "VECTOR_MAP_SERVICE_SUCCESS"
		if msg.status == 6:
			message = "VECTOR_MAP_SERVICE_FAILURE"
		if msg.status == 7:
			message = "GEOFENCE_CALL_SUCCESS"
		if msg.status == 8:	
			message = "GEOFENCE_CALL_FAILURE"
		if msg.status == 9:
			message = "PATH_PLANNER_SERVICE_SUCCESS"
		if msg.status == 10:
			message = "PATH_PLANNER_SERVICE_FAILED"
		if msg.status == 11:	
			message = "PATH_PLANNER_FAILED"
		if msg.status == 12:
			message = "PATH_PLANNER_REPOSITIONING_FAILED"
		if msg.status == 13:
			message = "POLYGONCONSTRAINT_SERVICE_SUCCESS"
		if msg.status == 14:	
			message = "POLYGONCONSTRAINT_SERVICE_FAILED"
		if msg.status == 15:
			message = "SMOOTHING_SERVICE_SUCCESS"
		if msg.status == 16:
			message = "SMOOTHING_SERVICE_FAILED"
		if msg.status == 17:
			message = "SMOOTHING_FAILED"
		if msg.status == 18:
			message = "DELTATVEC_SERVICE_SUCCESS"
		if msg.status == 19:
			message = "DELTATVEC_SERVICE_FAILURE"
		if msg.status == 20:	
			message = "DELTATVEC_CONSTRAINT_FAILURE"

		if self.missions_started:
			if self.active_robots["robot1"]["computing_task"] == 1:
				self.active_robots["robot1"]["report"] = message

	def robot2_computetaskstatus_callback(self,msg):
		if msg.status == 0:
			message = "COMPUTE_TASK_START"
		if msg.status == 1:
			message = "COMPUTE_TASK_SUCCESS"
		if msg.status == 2:	
			message = "INVALID_TARGET"
		if msg.status == 3:
			message = "INVALID_MAP"
		if msg.status == 4:
			message = "INVALID_START"
		if msg.status == 5:	
			message = "VECTOR_MAP_SERVICE_SUCCESS"
		if msg.status == 6:
			message = "VECTOR_MAP_SERVICE_FAILURE"
		if msg.status == 7:
			message = "GEOFENCE_CALL_SUCCESS"
		if msg.status == 8:	
			message = "GEOFENCE_CALL_FAILURE"
		if msg.status == 9:
			message = "PATH_PLANNER_SERVICE_SUCCESS"
		if msg.status == 10:
			message = "PATH_PLANNER_SERVICE_FAILED"
		if msg.status == 11:	
			message = "PATH_PLANNER_FAILED"
		if msg.status == 12:
			message = "PATH_PLANNER_REPOSITIONING_FAILED"
		if msg.status == 13:
			message = "POLYGONCONSTRAINT_SERVICE_SUCCESS"
		if msg.status == 14:	
			message = "POLYGONCONSTRAINT_SERVICE_FAILED"
		if msg.status == 15:
			message = "SMOOTHING_SERVICE_SUCCESS"
		if msg.status == 16:
			message = "SMOOTHING_SERVICE_FAILED"
		if msg.status == 17:
			message = "SMOOTHING_FAILED"
		if msg.status == 18:
			message = "DELTATVEC_SERVICE_SUCCESS"
		if msg.status == 19:
			message = "DELTATVEC_SERVICE_FAILURE"
		if msg.status == 20:	
			message = "DELTATVEC_CONSTRAINT_FAILURE"

		if self.missions_started:
			if self.active_robots["robot2"]["computing_task"] == 1:
				self.active_robots["robot2"]["report"] = message

	def robot3_computetaskstatus_callback(self,msg):
		if msg.status == 0:
			message = "COMPUTE_TASK_START"
		if msg.status == 1:
			message = "COMPUTE_TASK_SUCCESS"
		if msg.status == 2:	
			message = "INVALID_TARGET"
		if msg.status == 3:
			message = "INVALID_MAP"
		if msg.status == 4:
			message = "INVALID_START"
		if msg.status == 5:	
			message = "VECTOR_MAP_SERVICE_SUCCESS"
		if msg.status == 6:
			message = "VECTOR_MAP_SERVICE_FAILURE"
		if msg.status == 7:
			message = "GEOFENCE_CALL_SUCCESS"
		if msg.status == 8:	
			message = "GEOFENCE_CALL_FAILURE"
		if msg.status == 9:
			message = "PATH_PLANNER_SERVICE_SUCCESS"
		if msg.status == 10:
			message = "PATH_PLANNER_SERVICE_FAILED"
		if msg.status == 11:	
			message = "PATH_PLANNER_FAILED"
		if msg.status == 12:
			message = "PATH_PLANNER_REPOSITIONING_FAILED"
		if msg.status == 13:
			message = "POLYGONCONSTRAINT_SERVICE_SUCCESS"
		if msg.status == 14:	
			message = "POLYGONCONSTRAINT_SERVICE_FAILED"
		if msg.status == 15:
			message = "SMOOTHING_SERVICE_SUCCESS"
		if msg.status == 16:
			message = "SMOOTHING_SERVICE_FAILED"
		if msg.status == 17:
			message = "SMOOTHING_FAILED"
		if msg.status == 18:
			message = "DELTATVEC_SERVICE_SUCCESS"
		if msg.status == 19:
			message = "DELTATVEC_SERVICE_FAILURE"
		if msg.status == 20:	
			message = "DELTATVEC_CONSTRAINT_FAILURE"

		if self.missions_started:
			if self.active_robots["robot3"]["computing_task"] == 1:
				self.active_robots["robot3"]["report"] = message

	def robot4_computetaskstatus_callback(self,msg):
		if msg.status == 0:
			message = "COMPUTE_TASK_START"
		if msg.status == 1:
			message = "COMPUTE_TASK_SUCCESS"
		if msg.status == 2:	
			message = "INVALID_TARGET"
		if msg.status == 3:
			message = "INVALID_MAP"
		if msg.status == 4:
			message = "INVALID_START"
		if msg.status == 5:	
			message = "VECTOR_MAP_SERVICE_SUCCESS"
		if msg.status == 6:
			message = "VECTOR_MAP_SERVICE_FAILURE"
		if msg.status == 7:
			message = "GEOFENCE_CALL_SUCCESS"
		if msg.status == 8:	
			message = "GEOFENCE_CALL_FAILURE"
		if msg.status == 9:
			message = "PATH_PLANNER_SERVICE_SUCCESS"
		if msg.status == 10:
			message = "PATH_PLANNER_SERVICE_FAILED"
		if msg.status == 11:	
			message = "PATH_PLANNER_FAILED"
		if msg.status == 12:
			message = "PATH_PLANNER_REPOSITIONING_FAILED"
		if msg.status == 13:
			message = "POLYGONCONSTRAINT_SERVICE_SUCCESS"
		if msg.status == 14:	
			message = "POLYGONCONSTRAINT_SERVICE_FAILED"
		if msg.status == 15:
			message = "SMOOTHING_SERVICE_SUCCESS"
		if msg.status == 16:
			message = "SMOOTHING_SERVICE_FAILED"
		if msg.status == 17:
			message = "SMOOTHING_FAILED"
		if msg.status == 18:
			message = "DELTATVEC_SERVICE_SUCCESS"
		if msg.status == 19:
			message = "DELTATVEC_SERVICE_FAILURE"
		if msg.status == 20:	
			message = "DELTATVEC_CONSTRAINT_FAILURE"

		if self.missions_started:
			if self.active_robots["robot4"]["computing_task"] == 1:
				self.active_robots["robot4"]["report"] = message


	def robot5_computetaskstatus_callback(self,msg):
		if msg.status == 0:
			message = "COMPUTE_TASK_START"
		if msg.status == 1:
			message = "COMPUTE_TASK_SUCCESS"
		if msg.status == 2:	
			message = "INVALID_TARGET"
		if msg.status == 3:
			message = "INVALID_MAP"
		if msg.status == 4:
			message = "INVALID_START"
		if msg.status == 5:	
			message = "VECTOR_MAP_SERVICE_SUCCESS"
		if msg.status == 6:
			message = "VECTOR_MAP_SERVICE_FAILURE"
		if msg.status == 7:
			message = "GEOFENCE_CALL_SUCCESS"
		if msg.status == 8:	
			message = "GEOFENCE_CALL_FAILURE"
		if msg.status == 9:
			message = "PATH_PLANNER_SERVICE_SUCCESS"
		if msg.status == 10:
			message = "PATH_PLANNER_SERVICE_FAILED"
		if msg.status == 11:	
			message = "PATH_PLANNER_FAILED"
		if msg.status == 12:
			message = "PATH_PLANNER_REPOSITIONING_FAILED"
		if msg.status == 13:
			message = "POLYGONCONSTRAINT_SERVICE_SUCCESS"
		if msg.status == 14:	
			message = "POLYGONCONSTRAINT_SERVICE_FAILED"
		if msg.status == 15:
			message = "SMOOTHING_SERVICE_SUCCESS"
		if msg.status == 16:
			message = "SMOOTHING_SERVICE_FAILED"
		if msg.status == 17:
			message = "SMOOTHING_FAILED"
		if msg.status == 18:
			message = "DELTATVEC_SERVICE_SUCCESS"
		if msg.status == 19:
			message = "DELTATVEC_SERVICE_FAILURE"
		if msg.status == 20:	
			message = "DELTATVEC_CONSTRAINT_FAILURE"

		if self.missions_started:
			if self.active_robots["robot5"]["computing_task"] == 1:
				self.active_robots["robot5"]["report"] = message

	def robot6_computetaskstatus_callback(self,msg):
		if msg.status == 0:
			message = "COMPUTE_TASK_START"
		if msg.status == 1:
			message = "COMPUTE_TASK_SUCCESS"
		if msg.status == 2:	
			message = "INVALID_TARGET"
		if msg.status == 3:
			message = "INVALID_MAP"
		if msg.status == 4:
			message = "INVALID_START"
		if msg.status == 5:	
			message = "VECTOR_MAP_SERVICE_SUCCESS"
		if msg.status == 6:
			message = "VECTOR_MAP_SERVICE_FAILURE"
		if msg.status == 7:
			message = "GEOFENCE_CALL_SUCCESS"
		if msg.status == 8:	
			message = "GEOFENCE_CALL_FAILURE"
		if msg.status == 9:
			message = "PATH_PLANNER_SERVICE_SUCCESS"
		if msg.status == 10:
			message = "PATH_PLANNER_SERVICE_FAILED"
		if msg.status == 11:	
			message = "PATH_PLANNER_FAILED"
		if msg.status == 12:
			message = "PATH_PLANNER_REPOSITIONING_FAILED"
		if msg.status == 13:
			message = "POLYGONCONSTRAINT_SERVICE_SUCCESS"
		if msg.status == 14:	
			message = "POLYGONCONSTRAINT_SERVICE_FAILED"
		if msg.status == 15:
			message = "SMOOTHING_SERVICE_SUCCESS"
		if msg.status == 16:
			message = "SMOOTHING_SERVICE_FAILED"
		if msg.status == 17:
			message = "SMOOTHING_FAILED"
		if msg.status == 18:
			message = "DELTATVEC_SERVICE_SUCCESS"
		if msg.status == 19:
			message = "DELTATVEC_SERVICE_FAILURE"
		if msg.status == 20:	
			message = "DELTATVEC_CONSTRAINT_FAILURE"

		if self.missions_started:
			if self.active_robots["robot6"]["computing_task"] == 1:
				self.active_robots["robot6"]["report"] = message

	def robot7_computetaskstatus_callback(self,msg):
		if msg.status == 0:
			message = "COMPUTE_TASK_START"
		if msg.status == 1:
			message = "COMPUTE_TASK_SUCCESS"
		if msg.status == 2:	
			message = "INVALID_TARGET"
		if msg.status == 3:
			message = "INVALID_MAP"
		if msg.status == 4:
			message = "INVALID_START"
		if msg.status == 5:	
			message = "VECTOR_MAP_SERVICE_SUCCESS"
		if msg.status == 6:
			message = "VECTOR_MAP_SERVICE_FAILURE"
		if msg.status == 7:
			message = "GEOFENCE_CALL_SUCCESS"
		if msg.status == 8:	
			message = "GEOFENCE_CALL_FAILURE"
		if msg.status == 9:
			message = "PATH_PLANNER_SERVICE_SUCCESS"
		if msg.status == 10:
			message = "PATH_PLANNER_SERVICE_FAILED"
		if msg.status == 11:	
			message = "PATH_PLANNER_FAILED"
		if msg.status == 12:
			message = "PATH_PLANNER_REPOSITIONING_FAILED"
		if msg.status == 13:
			message = "POLYGONCONSTRAINT_SERVICE_SUCCESS"
		if msg.status == 14:	
			message = "POLYGONCONSTRAINT_SERVICE_FAILED"
		if msg.status == 15:
			message = "SMOOTHING_SERVICE_SUCCESS"
		if msg.status == 16:
			message = "SMOOTHING_SERVICE_FAILED"
		if msg.status == 17:
			message = "SMOOTHING_FAILED"
		if msg.status == 18:
			message = "DELTATVEC_SERVICE_SUCCESS"
		if msg.status == 19:
			message = "DELTATVEC_SERVICE_FAILURE"
		if msg.status == 20:	
			message = "DELTATVEC_CONSTRAINT_FAILURE"

		if self.missions_started:
			if self.active_robots["robot7"]["computing_task"] == 1:
				self.active_robots["robot7"]["report"] = message

	def robot8_computetaskstatus_callback(self,msg):
		if msg.status == 0:
			message = "COMPUTE_TASK_START"
		if msg.status == 1:
			message = "COMPUTE_TASK_SUCCESS"
		if msg.status == 2:	
			message = "INVALID_TARGET"
		if msg.status == 3:
			message = "INVALID_MAP"
		if msg.status == 4:
			message = "INVALID_START"
		if msg.status == 5:	
			message = "VECTOR_MAP_SERVICE_SUCCESS"
		if msg.status == 6:
			message = "VECTOR_MAP_SERVICE_FAILURE"
		if msg.status == 7:
			message = "GEOFENCE_CALL_SUCCESS"
		if msg.status == 8:	
			message = "GEOFENCE_CALL_FAILURE"
		if msg.status == 9:
			message = "PATH_PLANNER_SERVICE_SUCCESS"
		if msg.status == 10:
			message = "PATH_PLANNER_SERVICE_FAILED"
		if msg.status == 11:	
			message = "PATH_PLANNER_FAILED"
		if msg.status == 12:
			message = "PATH_PLANNER_REPOSITIONING_FAILED"
		if msg.status == 13:
			message = "POLYGONCONSTRAINT_SERVICE_SUCCESS"
		if msg.status == 14:	
			message = "POLYGONCONSTRAINT_SERVICE_FAILED"
		if msg.status == 15:
			message = "SMOOTHING_SERVICE_SUCCESS"
		if msg.status == 16:
			message = "SMOOTHING_SERVICE_FAILED"
		if msg.status == 17:
			message = "SMOOTHING_FAILED"
		if msg.status == 18:
			message = "DELTATVEC_SERVICE_SUCCESS"
		if msg.status == 19:
			message = "DELTATVEC_SERVICE_FAILURE"
		if msg.status == 20:	
			message = "DELTATVEC_CONSTRAINT_FAILURE"

		if self.missions_started:
			if self.active_robots["robot8"]["computing_task"] == 1:
				self.active_robots["robot8"]["report"] = message

	def robot9_computetaskstatus_callback(self,msg):
		if msg.status == 0:
			message = "COMPUTE_TASK_START"
		if msg.status == 1:
			message = "COMPUTE_TASK_SUCCESS"
		if msg.status == 2:	
			message = "INVALID_TARGET"
		if msg.status == 3:
			message = "INVALID_MAP"
		if msg.status == 4:
			message = "INVALID_START"
		if msg.status == 5:	
			message = "VECTOR_MAP_SERVICE_SUCCESS"
		if msg.status == 6:
			message = "VECTOR_MAP_SERVICE_FAILURE"
		if msg.status == 7:
			message = "GEOFENCE_CALL_SUCCESS"
		if msg.status == 8:	
			message = "GEOFENCE_CALL_FAILURE"
		if msg.status == 9:
			message = "PATH_PLANNER_SERVICE_SUCCESS"
		if msg.status == 10:
			message = "PATH_PLANNER_SERVICE_FAILED"
		if msg.status == 11:	
			message = "PATH_PLANNER_FAILED"
		if msg.status == 12:
			message = "PATH_PLANNER_REPOSITIONING_FAILED"
		if msg.status == 13:
			message = "POLYGONCONSTRAINT_SERVICE_SUCCESS"
		if msg.status == 14:	
			message = "POLYGONCONSTRAINT_SERVICE_FAILED"
		if msg.status == 15:
			message = "SMOOTHING_SERVICE_SUCCESS"
		if msg.status == 16:
			message = "SMOOTHING_SERVICE_FAILED"
		if msg.status == 17:
			message = "SMOOTHING_FAILED"
		if msg.status == 18:
			message = "DELTATVEC_SERVICE_SUCCESS"
		if msg.status == 19:
			message = "DELTATVEC_SERVICE_FAILURE"
		if msg.status == 20:	
			message = "DELTATVEC_CONSTRAINT_FAILURE"

		if self.missions_started:
			if self.active_robots["robot9"]["computing_task"] == 1:
				self.active_robots["robot9"]["report"] = message



	def robot1_pointclickgoal_callback(self,msg):
		robotgoal_msg = RobotTarget()
		robotgoal_msg.robot_id = 1
		goal_msg = PoseSteering()
		goal_msg.pose = msg.pose
		robotgoal_msg.goal = goal_msg
		robotgoal_msg.start_op.operation = 1
		robotgoal_msg.goal_op.operation = 1

		self.robot1_goal_pub.publish(robotgoal_msg)

	def robot2_pointclickgoal_callback(self,msg):
		robotgoal_msg = RobotTarget()
		robotgoal_msg.robot_id = 2
		goal_msg = PoseSteering()
		goal_msg.pose = msg.pose
		robotgoal_msg.goal = goal_msg
		robotgoal_msg.start_op.operation = 1
		robotgoal_msg.goal_op.operation = 1

		self.robot2_goal_pub.publish(robotgoal_msg)

	def robot3_pointclickgoal_callback(self,msg):
		robotgoal_msg = RobotTarget()
		robotgoal_msg.robot_id = 3
		goal_msg = PoseSteering()
		goal_msg.pose = msg.pose
		robotgoal_msg.goal = goal_msg
		robotgoal_msg.start_op.operation = 1
		robotgoal_msg.goal_op.operation = 1
		self.robot3_goal_pub.publish(robotgoal_msg)

	def robot4_pointclickgoal_callback(self,msg):
		robotgoal_msg = RobotTarget()
		robotgoal_msg.robot_id = 4
		goal_msg = PoseSteering()
		goal_msg.pose = msg.pose
		robotgoal_msg.goal = goal_msg
		robotgoal_msg.start_op.operation = 1
		robotgoal_msg.goal_op.operation = 1
		self.robot4_goal_pub.publish(robotgoal_msg)

	def robot5_pointclickgoal_callback(self,msg):
		robotgoal_msg = RobotTarget()
		robotgoal_msg.robot_id = 5
		goal_msg = PoseSteering()
		goal_msg.pose = msg.pose
		robotgoal_msg.goal = goal_msg
		robotgoal_msg.start_op.operation = 1
		robotgoal_msg.goal_op.operation = 1
		self.robot5_goal_pub.publish(robotgoal_msg)

	def robot6_pointclickgoal_callback(self,msg):
		robotgoal_msg = RobotTarget()
		robotgoal_msg.robot_id = 6
		goal_msg = PoseSteering()
		goal_msg.pose = msg.pose
		robotgoal_msg.goal = goal_msg
		robotgoal_msg.start_op.operation = 1
		robotgoal_msg.goal_op.operation = 1
		self.robot6_goal_pub.publish(robotgoal_msg)

	def robot7_pointclickgoal_callback(self,msg):
		robotgoal_msg = RobotTarget()
		robotgoal_msg.robot_id = 7
		goal_msg = PoseSteering()
		goal_msg.pose = msg.pose
		robotgoal_msg.goal = goal_msg
		robotgoal_msg.start_op.operation = 1
		robotgoal_msg.goal_op.operation = 1
		self.robot7_goal_pub.publish(robotgoal_msg)

	def robot8_pointclickgoal_callback(self,msg):
		robotgoal_msg = RobotTarget()
		robotgoal_msg.robot_id = 8
		goal_msg = PoseSteering()
		goal_msg.pose = msg.pose
		robotgoal_msg.goal = goal_msg
		robotgoal_msg.start_op.operation = 1
		robotgoal_msg.goal_op.operation = 1
		self.robot8_goal_pub.publish(robotgoal_msg)

	def robot9_pointclickgoal_callback(self,msg):
		robotgoal_msg = RobotTarget()
		robotgoal_msg.robot_id = 9
		goal_msg = PoseSteering()
		goal_msg.pose = msg.pose
		robotgoal_msg.goal = goal_msg
		robotgoal_msg.start_op.operation = 1
		robotgoal_msg.goal_op.operation = 1
		self.robot9_goal_pub.publish(robotgoal_msg)

if __name__ == '__main__':
	rospy.init_node('iliad_goal_manager_node', anonymous=True)
	igm = iliad_goal_manager()
