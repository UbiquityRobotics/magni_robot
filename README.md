
# Overview

This package contains launch files and configuration files for the magni robot.

## Launch files

### magni_bringup base.launch

This launch file runs the nodes needed to run a magni robot.
For more information, see [ubiquity_motor](https://github.com/UbiquityRobotics/ubiquity_motor)

    $ roslaunch magni_bringup base.launch

Once this is run, then the robot can be driven with teleop commands, for example:

    $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

### magni_nav aruco.launch

This launch file runs the fiducial marker based localization system using Raspberry Pi
camera.
For more information, see [fiducials](http://wiki.ros.org/fiducials) and
[raspicam_node](https://github.com/UbiquityRobotics/raspicam_node)

### magni_nav move_base.launch

This launch file runs navigation.
For more information, see [move_base](http://wiki.ros.org/move_base)

