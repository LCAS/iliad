^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hrsi_representation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2017-09-07)
------------------
* changelogs
* removed `'` as they are not allowed anymore (`#153 <https://github.com/strands-project/strands_hri/issues/153>`_)
  see https://lcas.lincoln.ac.uk/buildfarm/job/Kdev__strands_hri__ubuntu_xenial_amd64/3/console
* Contributors: Marc Hanheide

* removed `'` as they are not allowed anymore (`#153 <https://github.com/strands-project/strands_hri/issues/153>`_)
  see https://lcas.lincoln.ac.uk/buildfarm/job/Kdev__strands_hri__ubuntu_xenial_amd64/3/console
* Contributors: Marc Hanheide

0.1.2 (2016-11-03)
------------------

0.1.1 (2016-07-02)
------------------

0.1.0 (2016-01-20)
------------------
* Updating install targets for hrsi_representation and removing test due to completely changed output and message type, the rosbag used for the test does not contain the right data. Will have to record new test data.
* Online_qtc_creator nor publishes the original string coming out of the qsr_lib.
  Creating qtc states for human-robot and human-robot's goal
* Not transforming goal anymore
* adding goal republisher.
* Creating qtc relations between robot-human and robot_goal-human in online creator. Using this to learn mappings from observations to robot behaviour and for finally working particle filtering.
* Adjusted tests for new functionalities and migrated bag file to new people message
* More bug fixes in online creator, adjusting file writer.
* Adapting offline creator to qsr_lib 0.2
* Adding max buffer size
  only adding new points if they are different
  adding possible restriction of QSR generation angle.
* Adapting online_qtc_creator to new qsr_lib format of version 0.2
* Changing default distance for qtcbc_argprobd
* Adding qtcbc_argprobd to online qtc creator
* * Renaming hrsi_prediction package to hrsi_velocity_costmaps
  * Including QTCc states
  * Removed prediction part
  * Online qtc creator now also uses argprobd to publish distance values:
  * int: intimate space
  * per: personal space
  * soc: social space
  * pub: public space
  * Creating hrsi_state_prediction package that currently only uses a very simple model as a proof of concept.
* Adding the ability of only publishing the latest qtc state.
* Merge pull request `#136 <https://github.com/strands-project/strands_hri/issues/136>`_ from cdondrup/purge
  [hrsi_representation] Adding the ability of only publishing the latest qtc state.
* Adding the ability of only publishing the latest qtc state.
* Returning correct timestamp form online_qtc_creator
* Default param value should not be higher than max
  duh...
* Not using rosrun anymore for test
* Fixing broken install target
* Adding a test for the offline converter
* Creating an offline conversion script to read csv files and turn them into json qtc files.
* Removed hmm creator and renamed train to offline_qtc_creator.
  Also made some changes to only convert to qtc and attempt to train an hmm.
* Since this has been created before the actual merge of the qsr_lib changes, final adjustments had to be made.
* smoothing rate is not part of the parameters dictionary.
* Using new dynamic_args feature of qtc and qsr lib for parameter specification.
* Removing unnecessary prints.
* Adding correct smoothing and unittest
* Contributors: Christian Dondrup, Marc Hanheide

0.0.13 (2015-05-17)
-------------------
* Merge pull request `#112 <https://github.com/strands-project/strands_hri/issues/112>`_ from cdondrup/qsrs_for
  [hrsi_representation] Using 'qsrs_for' to prevent redundancy.
* 0.0.12
* updated changelogs
* Using 'qsrs_for' to prevent redundancy.
* Handling new string representation of qtc
* Contributors: Christian Dondrup, Jenkins

0.0.12 (2015-05-10)
-------------------

0.0.11 (2015-04-17)
-------------------

0.0.10 (2015-04-10)
-------------------

0.0.9 (2015-04-10)
------------------

0.0.8 (2015-04-02)
------------------
* Adjusted cmake and package files
* First working version of the onlie qtc creation.
* Training now works.
* Basic functionality of reading files and transforming the content into qtc
* Contributors: Christian Dondrup

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
