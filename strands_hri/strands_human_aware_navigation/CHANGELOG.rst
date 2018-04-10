^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package strands_human_aware_navigation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


0.2.01 (2018-04-10)
------------------
* MFC: parametrized several topics/frame_ids to allow scalation
* MFC: added simplegoal interface

0.2.0 (2017-09-07)
------------------
* changelogs
* Contributors: Marc Hanheide

0.1.2 (2016-11-03)
------------------

0.1.1 (2016-07-02)
------------------
* [human_aware_navigation] Exposing peopl_tracker topic via launch file
  Default has been set to used the filtered topic.
* Contributors: Christian Dondrup

0.1.0 (2016-01-20)
------------------
* Moving loaded parameters to correct namespace
* Adding site param file param
* Adding the custom dynamic reconfigure parameters to human_aware wrapper.
  @lucasb-eyer to test
* Contributors: Christian Dondrup

0.0.13 (2015-05-17)
-------------------
* Exposing human_aware_navigation params via launch file.
* 0.0.12
* updated changelogs
* Contributors: Christian Dondrup, Jenkins

0.0.12 (2015-05-10)
-------------------
* Adding ability of reconfiguring movebase and human_aware together. Taking new parameters into account in human_aware when resetting speeds.
* Contributors: Christian Dondrup

0.0.11 (2015-04-17)
-------------------

0.0.10 (2015-04-10)
-------------------

0.0.9 (2015-04-10)
------------------
* Changing default and min values for some of the params
  Min distance can now be 0.0. The problem is that, the robot will stop moving before the min distance is reached, because the transitional velocity will be too low for the planner to find a valid plan. This PR sets more conservative default values as wll to allow for easier testing and not having the robot stop because of @jsantos all the time ;)
  During the pre-deployment, we have to find a suitable set for AAF.
* Setting rot vel to 0 if trans vel is below magic number.
* Contributors: Christian Dondrup

0.0.7 (2014-12-01)
------------------
* [human_aware_navigation] Reading default velocities from move_base params on start
  Also not reconfiguring the min values any more. Those are never used but overwrite the set parameters.
* Contributors: Christian Dondrup

0.0.6 (2014-11-21)
------------------

0.0.5 (2014-11-11)
------------------

0.0.3 (2014-11-06)
------------------

0.0.2 (2014-10-31)
------------------

0.0.1 (2014-10-31)
------------------
* Removed calls to strands_head_orientation as those won't work in a release version.
* Added missing webpage
* Prepared strands_human_aware_navigation for release.
* Renamed strands_people_tracker to bayes_people_tracker
* strands_perception_people_msgs has been removed
* Adapting to new people_tracker message.
* Merge branch 'hydro-devel' into people_tracker
  Conflicts:
  strands_human_aware_navigation/src/scripts/human_aware_navigation.py
* Adapting the human_aware_velocity to the new topic name and type of the people tracker
* running head orientation while human_aware_navigation is running. Only if present. Changed default timeout of engaged server.
* Changing default parameters back to be more social.
* Changed default params
* Setting initial gaze goal for the nav_goal
  resetting the mode in the behaviour switch to prevent it from looking away.
* If human detected, look at human
  else look at nav goal
* Not changing rotational velocity.
* Hard coding "fast" values.
* changes to human aware navigation to handle new goals wothout outputting failure
* Cancelling the gaze goal if move_base is successful.
* Catching service exception for reconfigure.
  Using gaze_at_pose to lock at nav goal.
* Bug fix: subscribing before creating the action server leaad to errors due to checking for a non existing action server in the callback.
* Merge pull request #39 from cdondrup/hydro-devel
  Bug fix. Using dictionary of read parameters now. Needed changing in the...
* Bug fix. Using dictionary of read parameters now. Needed changing in the callback.:w
* Creating action server after every client has been created
* Minor bug fix: Preempting triggered an error because it still tried to set the goal as aborted.
  Also moved a lot of output to debug level or removed it completely.
* Piping through move_base result and feedback.
  Currently move_base does not publish a result though.
* Adapted human_aware_navigation to be used as navigation action server in topological map.
* Contributors: Bruno Lacerda, Christian Dondrup, cdondrup, strands
