^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package magni_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.1 (2022-03-18)
------------------
* generated_core.launch gets generated in /tmp fixing `#196 <https://github.com/UbiquityRobotics/magni_robot/issues/196>`_
* Contributors: Janez Cimerman

0.6.0 (2022-03-15)
------------------
* Slight documentation update
* Slightly improved architecture of robot.yaml
* Added redirection comments into base.yaml
* Pose covariance params setting fix
* Added missing wheel radius
* Added join pub stuff to launch_core.py
* Added board version to def settings and deleted sonars from launch core
* Adding new serial params into robot yaml, deleted base yaml after merge
* Put all prams form base to robot yaml, deleted base yaml
* Adding back in serial port and other serial params
* Back down pid_velocity to 0 and pid_integral to 5, that is less aggressive behavior
* Change base.yaml gear ratio and max pwm back to good values
* Adding new motor node and firmware parameters
* readme update
* Added per param robot.yaml corection, robot.yaml revert
* Max linear velocity increased to 1.1m/s
* converted whole robot.yaml into one-level thing for clearness
* lidar and camera enabled with one-level robot.yaml
* Contributors: Janez, Janez Cimerman, Mark Johnston, Teodor

0.5.1 (2021-08-06)
------------------
* Install magni-base.service here, but don't enable it
* Install the config dir so that default conf can be found in debs
* Contributors: Rohan Agrawal

0.5.0 (2021-07-30)
------------------
* Fixed dependancies for Focal/Noetic
* Readme update
* Change shebang to python3 in launch_core
* Moved extrinsics yamls from param/ to extrinsics/
* Complete rewrite of launch_core
* Use base_footprint for odometery, aruco, and move_base
* increase default acceleration limit related to `#148 <https://github.com/UbiquityRobotics/magni_robot/issues/148>`_
* Contributors: Janez, Janez Cimerman, Jim Vaughan, Mark Johnston, MoffKalast, Rohan Agrawal, Teodor, Vid Rijavec

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
