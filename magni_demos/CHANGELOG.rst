^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package magni_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.3 (2018-08-30)
------------------

0.4.2 (2018-08-26)
------------------

0.4.1 (2018-08-26)
------------------

0.4.0 (2018-08-26)
------------------

0.3.2 (2018-06-27)
------------------

0.3.1 (2018-06-26)
------------------

0.3.0 (2018-06-25)
------------------
* Add dnn_rotate demo (`#52 <https://github.com/UbiquityRobotics/magni_robot/issues/52>`_)
* Merge pull request `#51 <https://github.com/UbiquityRobotics/magni_robot/issues/51>`_ from rohbotics/launch_file_refactor
  Refactor Launch Files (and fix `#50 <https://github.com/UbiquityRobotics/magni_robot/issues/50>`_)
* make joystick and teleop launch aliaises again
* cleanup other demo launch files
* remove old demo launches
* renamed speech commands to simple_navigation
* remove roshub from default launch, make joystick a symlink again
* added roshub stuff
* Contributors: Jim Vaughan, Rohan Agrawal

0.2.4 (2017-12-23)
------------------

0.2.3 (2017-12-23)
------------------
* Only exec_depend to avoid pulling in unnecessary dependencies at build-time   (`#46 <https://github.com/UbiquityRobotics/magni_robot/issues/46>`_)
  * only exec_depend, cleanup package.xmls
* Contributors: Rohan Agrawal

0.2.2 (2017-12-22)
------------------
* Have both ssl and non-ssl version of rosbridge (`#39 <https://github.com/UbiquityRobotics/magni_robot/issues/39>`_)
  * have both ssl and non-ssl version of rosbridge
  * added tf2 web publisher to launch file
  * fix the base launch being commented out
* Contributors: Rohan Agrawal

0.2.1 (2017-10-28)
------------------
* Add launch file for fiducial_follow (`#40 <https://github.com/UbiquityRobotics/magni_robot/issues/40>`_)
  * Add launch file for fiducial_follow
  * Update camera for front facing
  * Use low res image for faster tracking
* Contributors: Jim Vaughan

0.2.0 (2017-08-19)
------------------
* add teleop launch that starts rosbridge. replaces joystick launch
* Contributors: Rohan Agrawal

0.1.1 (2017-07-04)
------------------
* Install launch/param dirs
* Contributors: Rohan Agrawal

0.1.0 (2017-06-17)
------------------
* Initial release
* Contributors: Rohan Agrawal
