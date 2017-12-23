^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package magni_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
