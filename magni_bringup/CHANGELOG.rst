^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package magni_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.3 (2018-08-30)
------------------

0.4.2 (2018-08-26)
------------------
* update postinst
* Contributors: Rohan Agrawal

0.4.1 (2018-08-26)
------------------
* made postinst executable
* Contributors: Rohan Agrawal

0.4.0 (2018-08-26)
------------------
* Wait for NTP before launching in Infrastructure mode
	- added postinst for chrony conf
	- do a smart wait for chrony
* Contributors: Rohan Agrawal

0.3.2 (2018-06-27)
------------------
* install core launch
* Contributors: Rohan Agrawal

0.3.1 (2018-06-26)
------------------
* use no sonars by default
* Contributors: Rohan Agrawal

0.3.0 (2018-06-25)
------------------
* Launch sonars if 'installed', but not by default (`#58 <https://github.com/UbiquityRobotics/magni_robot/issues/58>`_)
* Support getting robot configuration from a file in etc  (`#57 <https://github.com/UbiquityRobotics/magni_robot/issues/57>`_)
* Refactor Launch Files (and fix `#50 <https://github.com/UbiquityRobotics/magni_robot/issues/50>`_)
* launch the core launch from a python script that can do something smart
* start to re-organize the launch files, with base.launch being the boot up launch
  Also move the rosbridge stuff out to magni_teleop to be more modular.
* Contributors: Jim Vaughan, Rohan Agrawal

0.2.4 (2017-12-23)
------------------
* remove unneeded CMake requires
* Contributors: Rohan Agrawal

0.2.3 (2017-12-23)
------------------
* Only exec_depend to avoid pulling in unnecessary dependencies at build-time   (`#46 <https://github.com/UbiquityRobotics/magni_robot/issues/46>`_)
  * only exec_depend, cleanup package.xmls
* Contributors: Rohan Agrawal

0.2.2 (2017-12-22)
------------------

0.2.1 (2017-10-28)
------------------

0.2.0 (2017-08-19)
------------------
* use remap instead of republish and remove topic_tools dep
* Contributors: Rohan Agrawal

0.1.1 (2017-07-04)
------------------
* Install launch/param dirs
* Contributors: Rohan Agrawal

0.1.0 (2017-06-17)
------------------
* initial release
* Contributors: Jim Vaughan, Joe Landau, Kurt Christofferson, Rohan Agrawal
