^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hrsi_state_prediction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2017-09-07)
------------------
* changelogs
* Contributors: Marc Hanheide

0.1.2 (2016-11-03)
------------------
* Annotation tool for people trajectories (`#147 <https://github.com/strands-project/strands_hri/issues/147>`_)
  * Adding first experimental version of annotation tool
  * The annotation tool.
  * Quicker painting
  * Adding script to create csv files from database annotations to feed into offline qtc creator.
  * Fixing help messages
  * Creating a filter scripts that creates a new collection with only applicable entries.
  * Removed all the filtering from the annotation tool. Now done prior in the filtering script.
* Contributors: Christian Dondrup

0.1.1 (2016-07-02)
------------------
* Adding a default colour and using the right colour values. (`#144 <https://github.com/strands-project/strands_hri/issues/144>`_)
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
* state_predictor now uses new particle filter in qsr_prob_rep instead of own implementation.
* Removing unnecessary dynamic reconfigure and service options to load models.
* Removing hacky particle filter
* Publishing the result of the pf
* Adding a service to reset the filter bank of the particle filter. Creates a new filter even for previously observed humans.
* Making model dir a parameter and removing the simple rules instantiation.
* adding goal republisher.
* Creating qtc relations between robot-human and robot_goal-human in online creator. Using this to learn mappings from observations to robot behaviour and for finally working particle filtering.
* Adding more simple prediction models, using dyn reconf, making particle filter work... somehow.
* Adding simple particle filter based state prediction of qtcbc
* Adding prediction based on qtcbcs_argprobd to state predictor
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

* state_predictor now uses new particle filter in qsr_prob_rep instead of own implementation.
* Removing unnecessary dynamic reconfigure and service options to load models.
* Removing hacky particle filter
* Publishing the result of the pf
* Adding a service to reset the filter bank of the particle filter. Creates a new filter even for previously observed humans.
* Making model dir a parameter and removing the simple rules instantiation.
* adding goal republisher.
* Creating qtc relations between robot-human and robot_goal-human in online creator. Using this to learn mappings from observations to robot behaviour and for finally working particle filtering.
* Adding more simple prediction models, using dyn reconf, making particle filter work... somehow.
* Adding simple particle filter based state prediction of qtcbc
* Adding prediction based on qtcbcs_argprobd to state predictor
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
