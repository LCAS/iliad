#!/usr/bin/env python

'''
Taros19 experiments parser. Reads bags produced by experiment_setup.launch

- Bags are in folder "bags"
- Bag filenames follow the pattern S[scenario_id]-N[navigation_id]_YYYY-MM-DD-HH-mm-ss.bag
- scenario_id=[1-5]
  1) "Robot crossess left of human."
  2) "Robot crossess right of human."
  3) "Robot crossess is overtaken by human."
  4) "Robot crossess crosses human."
  5) "No human at all."
- navigation_id=[0-3]
  0) " NO Navigation - Possible collision."
  1) " DWA Navigation."
  2) " TEB Navigation."
  3) " MPC Navigation."

- Each bag file contains:
             /actor_pose              geometry_msgs/PoseStamped (world frame) (human interaction scenarios [S1 to S4])
             /robot1/robot_pose       geometry_msgs/PoseStamped (world frame) (always)

             /robot1/control/controller/reports   orunav_msgs/ControllerReport  (MPC navigation [N3])
             OR
             /robot1/move_base/status actionlib_msgs/GoalStatusArray (DWA or TEB navigation N1,N2)
             /isColliding (if there was a collision ...)             (No navigation [N0])

- We obtain 4 parameters
    path_length: using robot_pos
    completion_time: N1,N2: using timestamp from move_base/status
                     N3: using timestamp from control/report
    min_human_robot_distance: use scipy.spatial.distance.cdist, only in scenarios S1 to S4 and N not 0
    collision: only in N0

'''


import rosbag
import pandas as pd
import os
import numpy as np
from datetime import datetime


def curveLen(x,y):
    l = 0
    for i in range(1,len(x)):
        l = l + np.sqrt( (x[i]-x[i-1])**2 + (y[i]-y[i-1])**2)
    return l

def getBagFileNames(folderName):
    ans = []
    for file in os.listdir(folderName):
        if file.endswith(".bag"):
            ans.append(file)
    return ans


def asDatetime(rospyTimeArray):
    ans = []
    for rpT in rospyTimeArray:
        dt = pd.to_datetime(rpT.to_nsec(), unit='ns')
        ans.append(dt)
    return ans

def getRobotHumanDist(rx,ry,rt,hx,hy,ht):
    r_df = pd.DataFrame({'robot_x': rx,'robot_y': ry}, index=asDatetime(rt))
    h_df = pd.DataFrame({'human_x': hx, 'human_y': hy}, index=asDatetime(ht))
    result = r_df.join(h_df, how='outer')
    result.interpolate(inplace=True)
    result.fillna(method='bfill',inplace=True)
    result.fillna(method='ffill', inplace=True)

    result['dx'] = (result['robot_x'] - result['human_x'])
    result['dy'] = (result['robot_y'] - result['human_y'])
    result['d'] = np.sqrt((result['dx']**2) + (result['dy']**2) )
    #result['d'].plot()
    return result['d'].min()



# Main function.
if __name__ == '__main__':
    folder = '/home/manolofc/workspace/TAROS19/src/iliad/taros19_experiments/bags/'
    saveFile = '/home/manolofc/workspace/TAROS19/src/iliad/taros19_experiments/bags/results.csv'
    bagFiles = getBagFileNames(folder)

    # topic names in bag file
    actor_pose_topic = '/actor_pose'
    robot_pose_topic = '/robot1/robot_pose'
    collision_topic = '/isColliding'

    robot_status_mpc_topic = '/robot1/control/controller/reports'
    robot_status_ros_topic = '/robot1/move_base/status'

    saveOutput = True


    # results
    scenario_ids = []
    navigation_ids = []
    recording_dates = []
    completion_times = []
    path_lengths = []
    min_human_robot_distances = []
    collisions = []

    for bagFile in bagFiles:


            #bagFile = bagFiles[1]  # N1
            #bagFile = bagFiles[0] # N2
            #bagFile = bagFiles[9] # N3

            print "Processing bag: "+bagFile

            scenario_id = bagFile[1]
            scenario_ids.append(scenario_id)
            navigation_id = bagFile[4]
            navigation_ids.append(navigation_id)
            recording_date_str = bagFile[6:-4]

            #if scenario_id == '3':
            bag = rosbag.Bag(folder+bagFile)

            recording_date = datetime.strptime(recording_date_str, '%Y-%m-%d-%H-%M-%S')
            recording_dates.append(recording_date)
            robot_pose_x = []
            robot_pose_y = []
            robot_pose_t = []

            human_pose_x = []
            human_pose_y = []
            human_pose_t = []

            status = []
            status_t = []

            # main loop
            endTime = -1
            hasCollided = False
            robotRunning = False
            t = None
            startTime = -1
            for topic, msg, t in bag.read_messages():
                if startTime == -1:
                    startTime = t
                if topic == actor_pose_topic:
                    human_pose_t.append(t)
                    human_pose_x.append(msg.pose.position.x)
                    human_pose_y.append(msg.pose.position.y)
                if topic == robot_pose_topic:
                    robot_pose_t.append(t)
                    robot_pose_x.append(msg.pose.position.x)
                    robot_pose_y.append(msg.pose.position.y)
                if topic == collision_topic:
                    hasCollided = True
                if topic == robot_status_mpc_topic:
                    status.append(msg.status)
                    status_t.append(t)
                    # goal under execution when status == 3
                    if (status[-1] == 3):
                        robotRunning = True
                    # goal achieved when status goes back to 1
                    if (status[-1] == 1) and (robotRunning):
                        endTime = status_t[-1]
                        break

                if topic == robot_status_ros_topic:
                    numGoals =len(msg.status_list)
                    if numGoals > 1:
                        print "FOCUSING JUST ON FIRST GOAL !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                    if numGoals>0:
                        status.append(msg.status_list[0].status)
                        status_t.append(t)
                        # goal achieved is status == 3
                        if (status[-1] ==3):
                            endTime = status_t[-1]
                            break
            # close bag
            bag.close()

            collisions.append(hasCollided)
            if endTime!=-1:
                completion_time =  (endTime-startTime).to_sec()
            else:
                completion_time = np.nan
            print "Test lasted "+str(completion_time)+" secs"
            completion_times.append(completion_time)
            path_length = curveLen(robot_pose_x,robot_pose_y)
            path_lengths.append(path_length)

            if scenario_id!=5:
                robot_human_dist = getRobotHumanDist(robot_pose_x,robot_pose_y,robot_pose_t,
                                                     human_pose_x,human_pose_y,human_pose_t)
            else:
                robot_human_dist = np.nan

            min_human_robot_distances.append(robot_human_dist)

    print "Saving results"
    df = pd.DataFrame({'scenario_ids' : scenario_ids,     'navigation_ids' : navigation_ids,     'completion_times' : completion_times,     'path_lengths' : path_lengths,     'min_human_robot_distances' : min_human_robot_distances,     'collisions' : collisions,     'recording_dates' : recording_dates} )
    df.sort_values(by=['scenario_ids', 'navigation_ids'],inplace=True)

    if saveOutput:
        df.to_csv(saveFile, index=True , columns=['scenario_ids' ,'navigation_ids',  'completion_times',  'path_lengths',     'min_human_robot_distances' ,     'collisions',     'recording_dates' ])
