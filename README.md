
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

# Software contents of the Magni robot:

Repositories containing this software are listed below.
A list of packages is not available yet.

## Software that enables core hardware functionality:

    https://github.com/UbiquityRobotics/ubiquity_motor     - Makes the Motors work; Package; no UI; no documentation

    https://github.com/UbiquityRobotics/raspicam_node      - Makes the Cameras work; Package; Calibration UI (documented)--is this exposed to User? 

    Motor controller firmware -- not to be released.

## Software that enables core system functionality

    https://github.com/UbiquityRobotics/magni_robot         - Launch files and other core software functionality; several packages; no UI; no documentation

    https://github.com/UbiquityRobotics/fiducials           - Enables fiducial localization; several packages; no intrinsic UI; UI via Rviz, which is documented in wiki 

    https://github.com/UbiquityRobotics/move_basic          - Simple path planner; Package; no intrinsic UI; some UI via Rviz; external documentation
    
    https://github.com/rohbotics/pifi                       Wifi provisioning tools for Robots with Raspberry Pis; UI; documented

## Robot Commander

    https://github.com/UbiquityRobotics/speech_commands        Provides voice and screen UI; web page; documented

    https://github.com/UbiquityRobotics/Robot_Commander        Provides voice and screen UI; android app; documented

    https://github.com/UbiquityRobotics/paramdump            Provides persistence for waypoints; Package; no documentation


## Applications Software

    https://github.com/UbiquityRobotics/demos             - fiducial_follow and partybot; former is a package, latter not; Both are UIs; no documentation
