
# Overview

This package contains launch files and configuration files for the Magni robot.

## Launch files

### magni_description magni_description.launch.py
Internal launch file for making the URDF with robot description load and robot_state_publisher work.

**robot_description**: all robot static and dynamic transforms. It gets its camera and lidar extrinsics from either `~/.ros/extrinsics/<SENSOR>_extrinsics_<POSITION>.yaml` OR `magni_description/urdf/extrinsics/<SENSOR>_extrinsics_<POSITION>.yaml` with respective priorities. Which extrinsics are taken can be controlled through robot.yaml which sets the `<SENSOR>` and `<POSITION>`.

**TODO:** Port other parts of this package from ROS1 to ROS2