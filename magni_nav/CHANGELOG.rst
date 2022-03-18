^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package magni_nav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2022-03-18)
------------------

0.6.0 (2022-03-15)
------------------

0.5.1 (2021-08-06)
------------------

0.5.0 (2021-07-30)
------------------
* Pass in fid len (`#116 <https://github.com/UbiquityRobotics/magni_robot/issues/116>`_)
* Added license files to all magni packages
* Localization error monitor created. Spiral trajectory cmd_vel script created.  (`#92 <https://github.com/UbiquityRobotics/magni_robot/issues/92>`_)
  * Fiducials world created
  * Paper background added to markers
  * Localization monitor created. cmd_vel trajectory publisher created.
  * Plot update rate configured. Refactoring.
  * Localization data errors print added.
* Fix side dist PID params
* Move basic launch file
* Add params for PID control of lateral error
* Use base_footprint in move_basic
* Use base_footprint for odometery, aruco, and move_base
* Contributors: Janez Cimerman, Jim Vaughan, Petro Shmigelskyi, Rohan Agrawal

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
* Add footprint (`#48 <https://github.com/UbiquityRobotics/magni_robot/issues/48>`_)
* Remove launch file for old fiducials. (`#47 <https://github.com/UbiquityRobotics/magni_robot/issues/47>`_)
  * Remove launch file for old fiducials.
  * Remove fiducial_detect exec_depend
* Contributors: Jim Vaughan

0.2.4 (2017-12-23)
------------------

0.2.3 (2017-12-23)
------------------
* Only exec_depend to avoid pulling in unnecessary dependencies at build-time   (`#46 <https://github.com/UbiquityRobotics/magni_robot/issues/46>`_)
  * only exec_depend, cleanup package.xmls
* Contributors: Rohan Agrawal

0.2.2 (2017-12-22)
------------------
* fix magni_nav metapackage dep warnings
* Contributors: Rohan Agrawal

0.2.1 (2017-10-28)
------------------

0.2.0 (2017-08-19)
------------------

0.1.1 (2017-07-04)
------------------
* Install launch/param dirs
* Contributors: Rohan Agrawal

0.1.0 (2017-06-17)
------------------
* Initial release
* Contributors: Jim Vaughan, Rohan Agrawal
