#!/usr/bin/env python

'''
NCFM tiny bags trajectory plotter.
Reads (tiny) bags made from NCFM experiments and plots human and robot trajectories.


/robot5/move_base/current_goal_odom
/robot5/online_qtc_creator/qtc_array
/robot5/people_tracker/positions
/robot5/robot_pose_odom

Task descriptions

1.1  Human crosses left-right
1.2  Human crosses right-left
2.1  Human passes by in open space
2.2  Human passes by in closed space
3.1  Human overtakes in open space
3.2  Human overtakes in closed space
4.1  Stationary human, robot overtakes in open space
4.2  Stationary human, robot overtakes in closed open space


'''


import rosbag
import pandas as pd
import os
import numpy as np
from geometry_msgs.msg import Point
from datetime import datetime
import matplotlib.pyplot as plt




def getBagFileNames(folderName):
    '''

    :param folderName: absolute folder route
    :return: list of bag files in that folder
    '''
    ans = []
    for file in os.listdir(folderName):
        if file.endswith(".bag"):
            ans.append(file)
    return ans



# ...................................................................................................


# Main function.
if __name__ == '__main__':
    bagsFolder = '/home/manolofc/iliad/NCFM_HUMAN_BAGS_processed3/'
    bagFiles = getBagFileNames(bagsFolder)
    actor_pose_topic='/robot5/people_tracker/positions'
    robot_pose_topic='/robot5/robot_pose_odom'
    goal_topic='/robot5/move_base/current_goal_odom'

    desc_dict={'1.1': 'Human crosses left-right', '1.2': 'Human crosses right-left', '2.1': 'Human passes by in open space', '2.2': 'Human passes by in closed space', '3.1': 'Human overtakes in open space', '3.2': 'Human overtakes in closed space', '4.1': 'Stationary human, robot overtakes in open space', '4.2': 'Stationary human, robot overtakes in closed open space',}


    # Process each bag  ........................................................

    for bagFile in bagFiles:
        scenario_id = bagFile[1]
        test_id = bagFile[4]
        subtest_id = bagFile[6]
        attempt_id = bagFile[9]
        recording_date_str = bagFile[16:-4]

        desc= desc_dict[test_id+'.'+subtest_id]
        desc = desc+' Atempt '+attempt_id
        print ".................................."
        print "Processing bag: "+bagFile

        bag = rosbag.Bag(bagsFolder+bagFile)
        goal_pose_t = []
        goal_pose_x = []
        goal_pose_y = []

        human_pose_t = []
        human_pose_x = []
        human_pose_y = []

        robot_pose_t = []
        robot_pose_x = []
        robot_pose_y = []

        for topic, msg, t in bag.read_messages():
            if topic == actor_pose_topic:
                if len(msg.poses)>0:
                    human_pose_t.append(t)
                    human_pose_x.append(msg.poses[0].position.x)
                    human_pose_y.append(msg.poses[0].position.y)
            if topic == robot_pose_topic:
                robot_pose_t.append(t)
                robot_pose_x.append(msg.pose.position.x)
                robot_pose_y.append(msg.pose.position.y)
            if topic == goal_topic:
                goal_pose_t.append(t)
                goal_pose_x.append(msg.pose.position.x)
                goal_pose_y.append(msg.pose.position.y)

        plt.plot(robot_pose_x, robot_pose_y, '.', label='robot')
        plt.plot(human_pose_x, human_pose_y, '.',label='human')
        plt.title(desc)
        plt.axis('equal')
        plt.show()
        # close bag
        bag.close()
        # Metrics for the bag ..................................................

