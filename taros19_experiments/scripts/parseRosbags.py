#!/usr/bin/env python

'''
Taros19 experiments parser. Reads bags produced by experiment_setup.launch

- Bags are in folder "bags"
- Bag filenames follow the pattern S[scenario_id]-N[navigation_id]_YYYY-MM-DD-HH-mm-ss.bag
- scenario_id=[1-5]
  1) "Robot crossess left of human."
  2) "Robot crossess right of human."
  3) "Robot crossess is overtaken by human."
  4) "Robot goes into human."
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
from geometry_msgs.msg import Point
from datetime import datetime
import matplotlib.pyplot as plt

def curveLen(x, y):
    l = 0
    for i in range(1, len(x)):
        l = l + np.sqrt((x[i]-x[i-1])**2 + (y[i]-y[i-1])**2)
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


def getRobotHumanDist(rx, ry, rt, hx, hy, ht):
    r_df = pd.DataFrame({'robot_x': rx, 'robot_y': ry}, index=asDatetime(rt))
    h_df = pd.DataFrame({'human_x': hx, 'human_y': hy}, index=asDatetime(ht))
    result = r_df.join(h_df, how='outer')
    result.interpolate(inplace=True)
    result.fillna(method='bfill', inplace=True)
    result.fillna(method='ffill', inplace=True)

    result['dx'] = (result['robot_x'] - result['human_x'])
    result['dy'] = (result['robot_y'] - result['human_y'])
    result['d'] = np.sqrt((result['dx']**2) + (result['dy']**2))
    # result['d'].plot()
    ans = result['d'].min()
    return ans


def dist(position1, position2):
    d = np.sqrt((position1.x-position2.x)**2 + (position1.y-position2.y)**2)
    return d


def getFirstDifferentIndex(x, y):
    firstIndex = 0
    d_len = len(x)
    found = not ((x[firstIndex] == x[firstIndex + 1]) and (y[firstIndex] == y[firstIndex + 1]))
    while not found:
        if firstIndex < d_len - 2:
            firstIndex = firstIndex + 1
            found = not ((x[firstIndex] == x[firstIndex + 1]) and (y[firstIndex] == y[firstIndex + 1]))
        else:
            firstIndex = 0
            found = True
    return (firstIndex, found)


def getLastDifferentIndex(x, y):
    d_len = len(x)
    lastIndex = d_len - 1
    found = not ((x[lastIndex] == x[lastIndex - 1]) and (y[lastIndex] == y[lastIndex - 1]))
    while not found:
        if lastIndex > 1:
            lastIndex = lastIndex - 1
            found = not ((x[lastIndex] == x[lastIndex - 1]) and (y[lastIndex] == y[lastIndex - 1]))
        else:
            lastIndex = d_len - 1
            found = True
    return (lastIndex, found)


def getSpeedDF(x0, y0, t0):
    x = np.array(x0)
    y = np.array(y0)
    (firstIndex, found) = getFirstDifferentIndex(x, y)
    (lastIndex, found) = getLastDifferentIndex(x, y)

    x = x[firstIndex:lastIndex + 1]
    y = y[firstIndex:lastIndex + 1]
    t = t0[firstIndex:lastIndex + 1]

    df = pd.DataFrame({'x': x, 'y': y}, index=asDatetime(t))
    ddf = df.diff()
    ddf['v'] = np.sqrt((ddf.x.values) ** 2 + (ddf.y.values) ** 2)
    return ddf['v']


def getAvSpeed(x, y, t):
    v = getSpeedDF(x, y, t)
    vm = v.mean()
    return vm


def getSpeedOld(x, y, t):
    t2 = np.array([ti.to_sec() for ti in t])
    t3 = t2[1:]
    it = (np.array(t2[1:]) - np.array(t2[:-1]))

    ix = np.array(x[1:]) - np.array(x[:-1])
    iy = np.array(y[1:]) - np.array(y[:-1])

    # several points are reported on the same time stamp???
    goodIndexes = it != 0
    it = it[goodIndexes]
    t3 = t3[goodIndexes]
    ix = ix[goodIndexes]
    iy = iy[goodIndexes]

    d = []
    for i in range(len(ix)):
        di = np.sqrt((ix[i]) ** 2 + (iy[i]) ** 2)
        d.append(di)

    (firstIndex, found) = getFirstDifferentIndex(ix, iy)
    (lastIndex, found) = getLastDifferentIndex(x, y)

    d = d[firstIndex:lastIndex + 1]
    it = it[firstIndex:lastIndex + 1]
    v = (np.array(d)) / it

    return v


def getAvSpeedOldOld(x, y, t):
    it = (t[-1] - t[0]).to_sec()
    ix = x[-1] - x[0]
    iy = y[-1] - y[0]

    v = np.sqrt((ix) ** 2 + (iy) ** 2) / it

    return v


def getMeanSpeed(x, y, t):
    (firstIndex, found) = getFirstDifferentIndex(x, y)
    (lastIndex, found) = getLastDifferentIndex(x, y)
    dx = x[lastIndex] - x[firstIndex]
    dy = y[lastIndex] - y[firstIndex]
    dt = (t[lastIndex] - t[firstIndex]).to_sec()

    v = np.sqrt((dx) ** 2 + (dy) ** 2) / dt
    return v

# ...................................................................................................
# ...................................................................................................
# ...................................................................................................
# ...................................................................................................


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

    saveOutput = False

    generic_goal = Point()
    generic_goal.x = -20
    generic_goal.y = -3.3

    # results
    scenario_ids = []
    navigation_ids = []
    recording_dates = []
    completion_times = []
    path_lengths = []
    min_human_robot_distances = []
    collisions = []
    robot_speeds = []
    human_speeds = []
    bagFileNames = []

    #if not saveOutput:
    #    bagFile = 'S3-N0_2019-01-28-15-53-04.bag'

    # Process each bag  ........................................................

    for bagFile in bagFiles:
      scenario_id = bagFile[1]
      navigation_id = bagFile[4]
      if (scenario_id=='2') and (navigation_id =='3'):
            # bagFile = bagFiles[1]  # N1
            # bagFile = bagFiles[0] # N2
            # bagFile = bagFiles[9] # N3

        print ".................................."
        print "Processing bag: "+bagFile

        scenario_ids.append(scenario_id)

        navigation_ids.append(navigation_id)
        bagFileNames.append(bagFile)


        recording_date_str = bagFile[6:-4]
        minTargetDist=1
        # if scenario_id == '3':
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

        # Process bag ..........................................................
        endTime = -1
        altEndIndex_r = -1
        altEndIndex_h = -1
        hasCollided = False
        collisionTime = -1
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
                target_dist = dist(msg.pose.position, generic_goal)
                if target_dist < minTargetDist:
                    minTargetDist = target_dist
                    altEndTime = t
                    altEndIndex_r = len(robot_pose_t)
                    # they have different indexes!!!
                    altEndIndex_h = len(human_pose_t)
            if topic == collision_topic:
                hasCollided = True
                collisionTime = t
                endTime = t
                print "Test finished abruptly after colision!!!!!"
                break
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
                numGoals = len(msg.status_list)
                if numGoals > 1:
                    print "FOCUSING JUST ON FIRST GOAL !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                if numGoals > 0:
                    status.append(msg.status_list[0].status)
                    status_t.append(t)
                    # goal achieved is status == 3
                    if (status[-1] == 3):
                        endTime = status_t[-1]
                        break
        # close bag
        bag.close()
        # Metrics for the bag ..................................................

        # METRIC: COMPLETION TIME
        if endTime != -1:
            completion_time = (endTime-startTime).to_sec()
        elif altEndIndex_r != -1:
            print "No navigation goal reported reaching goal. Closest distance to it was "+str(minTargetDist)
            # we got close to target, but state didn't reported. Mostly why there was no nav working
            completion_time = (altEndTime-startTime).to_sec()
            robot_pose_x = robot_pose_x[0:altEndIndex_r]
            robot_pose_y = robot_pose_y[0:altEndIndex_r]
            robot_pose_t = robot_pose_t[0:altEndIndex_r]
            human_pose_x = human_pose_x[0:altEndIndex_h]
            human_pose_y = human_pose_y[0:altEndIndex_h]
            human_pose_t = human_pose_t[0:altEndIndex_h]
        else:
            completion_time = np.nan
            print "Test didnt finish, nor went close to final goal!!!!!"

        print "Test lasted "+str(completion_time)+" secs"
        completion_times.append(completion_time)

        # METRIC: COLLISIONS
        collisions.append(hasCollided)

        # METRIC: PATH LENGTH
        path_length = curveLen(robot_pose_x, robot_pose_y)
        path_lengths.append(path_length)

        # METRIC: DISTANCE TO HUMAN
        if scenario_id != '5':
            robot_human_dist = getRobotHumanDist(robot_pose_x, robot_pose_y, robot_pose_t,
                                                 human_pose_x, human_pose_y, human_pose_t)
            if human_pose_x == []:
                print "\n\nxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
                print "No human pose! Something is wrong in recording "+bagFile
                print "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n\n"
        else:
            robot_human_dist = np.nan

        min_human_robot_distances.append(robot_human_dist)

        # METRIC: SPEEDS
        rs = getMeanSpeed(robot_pose_x, robot_pose_y, robot_pose_t)
        print "av. robot speed: "+str(rs)
        robot_speeds.append(rs)
        if scenario_id != '5':
            hs = getMeanSpeed(human_pose_x, human_pose_y, human_pose_t)
            human_speeds.append(hs)
            print "av. human speed: " + str(hs)
            if not np.isfinite(hs) or (hs == 0.0):
                print "\n\nxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
                print "No human motion! something is wrong in recording "+bagFile
                print "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n\n"
                plt.plot(human_pose_x,human_pose_y)
                plt.plot(robot_pose_x,robot_pose_y,'r')
                plt.axis('equal')
                plt.show()

        else:
            human_speeds.append(np.nan)
            print "no human speed here "
    # Save metrics for all bags and exit .......................................

    print "Saving results"
    df = pd.DataFrame({'scenario_ids': scenario_ids,
                       'navigation_ids': navigation_ids,
                       'completion_times': completion_times,
                       'path_lengths': path_lengths,
                       'robot_speeds': robot_speeds,
                       'human_speeds': human_speeds,
                       'min_human_robot_distances': min_human_robot_distances,
                       'collisions': collisions,
                       'bagfilenames': bagFileNames,
                       'recording_dates': recording_dates})

    df.sort_values(by=['scenario_ids', 'navigation_ids'], inplace=True)

    if saveOutput:
        df.to_csv(saveFile, index=True, columns=['scenario_ids', 'navigation_ids',  'completion_times',
                                                 'path_lengths', 'robot_speeds','human_speeds',     'min_human_robot_distances',     'collisions',     'recording_dates', 'bagfilenames'])
