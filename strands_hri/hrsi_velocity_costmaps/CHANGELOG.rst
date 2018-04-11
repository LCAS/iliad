^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hrsi_velocity_costmaps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2017-09-07)
------------------
* changelogs
* Contributors: Marc Hanheide

0.1.2 (2016-11-03)
------------------
* [Velocity Costmaps] Catching identity change during interaction
  If a person changes their tracking id while the best model is classified and the belief generated, the UUID for costmap generation is not the same as in the current list of tracked people any more. If this is the case, the velo map is just not created.
* Contributors: Christian Dondrup

0.1.1 (2016-07-02)
------------------
* Adding a marker showing for whom the velocity costmap is created. (`#145 <https://github.com/strands-project/strands_hri/issues/145>`_)
  * Adding a marker showing for whom the velocity costmap is created.
  * Using same frame as people tracker message.
* Visualising classification results from state prediction and adding models (`#143 <https://github.com/strands-project/strands_hri/issues/143>`_)
  * Adding models for passby and crossing
  * Fixed an error where the models were overridden every time a new one was created.
  * If move_base is not running, the velocity_costmap_creator will now retry to get the parameter instead of dying.
  * Adding config file to specify colours for classification results to visualise the output.
  * Adding dedicated launch file for state_prediction.
  * Using state_prediction launch file in hrsi_launch
  * Adding install targets for new directories.
  * Adding visualisation markers for classification results.
* Contributors: Christian Dondrup

0.1.0 (2016-01-20)
------------------
* velocity costmap server publishes to `/velocity_costmap` now and takes the qsr_lib style qtc strings as input.
  Also updating cmake and package file.
* Creating qtc relations between robot-human and robot_goal-human in online creator. Using this to learn mappings from observations to robot behaviour and for finally working particle filtering.
* Minor changes: no more gradual costs in qtcb
* Running velocity costmap creation loop only if not '?' given.
* Adding gradual costs to velocity costmaps. This way not all of the free area has 0 costs but forces the robot to stay in the middle of the area if possible given the other costs.
* * Renaming hrsi_prediction package to hrsi_velocity_costmaps
  * Including QTCc states
  * Removed prediction part
  * Online qtc creator now also uses argprobd to publish distance values:
  * int: intimate space
  * per: personal space
  * soc: social space
  * pub: public space
  * Creating hrsi_state_prediction package that currently only uses a very simple model as a proof of concept.
* Contributors: Christian Dondrup

* velocity costmap server publishes to `/velocity_costmap` now and takes the qsr_lib style qtc strings as input.
  Also updating cmake and package file.
* Creating qtc relations between robot-human and robot_goal-human in online creator. Using this to learn mappings from observations to robot behaviour and for finally working particle filtering.
* Minor changes: no more gradual costs in qtcb
* Running velocity costmap creation loop only if not '?' given.
* Adding gradual costs to velocity costmaps. This way not all of the free area has 0 costs but forces the robot to stay in the middle of the area if possible given the other costs.
* * Renaming hrsi_prediction package to hrsi_velocity_costmaps
  * Including QTCc states
  * Removed prediction part
  * Online qtc creator now also uses argprobd to publish distance values:
  * int: intimate space
  * per: personal space
  * soc: social space
  * pub: public space
  * Creating hrsi_state_prediction package that currently only uses a very simple model as a proof of concept.
* Contributors: Christian Dondrup

0.0.13 (2015-05-17)
-------------------

0.0.12 (2015-05-10)
-------------------

0.0.11 (2015-04-17)
-------------------

0.0.10 (2015-04-10 11:06)
-------------------------

0.0.9 (2015-04-10 10:21)
------------------------

0.0.8 (2015-04-02)
------------------

0.0.7 (2014-12-01)
------------------

0.0.6 (2014-11-21)
------------------

0.0.5 (2014-11-11 14:00)
------------------------

0.0.4 (2014-11-11 12:20)
------------------------

0.0.3 (2014-11-06)
------------------

0.0.2 (2014-10-31 18:55)
------------------------

0.0.1 (2014-10-31 17:17)
------------------------
