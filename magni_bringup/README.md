# Magni bringup

**Robot startup procedure:**
Magni robots start ros core and a set of base nodes on boot. This is done in the following order:

`systemctl start magni-base.service`:
  - env variables
  - `base.launch`:
    - `logitech.launch`
    - `rosbridge.launch`
    - `launch_core.py`:
      - `/tmp/core.launch`

1. using `systemctl` the magni-base service file (`/usr/sbin/magni-base.service`) is ran:
     - sources ros overlay and working workspaces
     - sets up environment variables (like ROS_HOSTNAME, and ROS_MASTER_URI)
     - placeholder for anything that needs to happen on boot
     - launches `base.launch`

2. `base.launch` launches robot base nodes:
     - `logitech.launch`
     - `rosbridge.launch` 
     - placeholder for other nodes that are NOT VARIABLE (don't change with robot config)
     - starts `launch_core.py`

3. `launch_core.py` generates and launches `generated_core.launch` (by default in `/tmp/generated_core.launch` but can be changed with [--launch_generate_path](https://github.com/UbiquityRobotics/magni_robot/blob/c8a130c8caf8ba3b26974da581bb97b2957da20b/magni_bringup/scripts/launch_core.py#L224) argument) which starts up the rest of the robot nodes that are variable (change with robot config):
     - **motor_node**: runs ubiquity_motor node with the parameters extracted from either `/etc/ubiquity/robot.yaml` OR `magni_bringup/param/default_robot.yaml` with respective priorities. If there are some parameters missing in `/etc/ubiquity/robot.yaml` they are taken individually from `magni_bringup/param/default_robot.yaml`. Which specific parameter was taken from where can be seen on printout of the `launch_core.py` when run manually.
     - **robot_description**: all robot static and dynamic transforms. It gets its camera and lidar extrinsics from either `~/.ros/extrinsics/<SENSOR>_extrinsics_<POSITION>.yaml` OR `magni_description/extrinsics/<SENSOR>_extrinsics_<POSITION>.yaml` with respective priorities. Which extrinsics are taken can be controlled through robot.yaml which sets the `<SENSOR>` and `<POSITION>`.
     - other nodes like `controller_spawner`, `diagnostic_aggregator`, `oled_display_node`,... are also launched. Exactly what gets launched can be seen in `magni_bringup/launch/core_launch.em` from which `core.launch` is generated. Any addition to `core.launch` should be added inside the `core_launch.em` file (an example of how parameters should be added to `robot.yaml`, `core_launch.em` and `launch_core.py` can be seen [here](https://github.com/UbiquityRobotics/magni_robot/pull/199))

4. There is also a `magni_bringup/scripts/ros_log_clean.bash` present that takes care of deleting the log files so they don't end up taking too much space. This can be included into `magni-base.service` so the check and deletion can be done on every boot.


**Useful commands for debugging**

 - If new additions need to be added to `generated_core.launch`, add them into `magni_bringup/param/core_launch.em`, then edit the launch_core.py accordingly. You can then run the commands:

 - `python3 src/magni_robot/magni_bringup/scripts/launch_core.py --debug` uses `launch_core.py` to create the `core.launch` but does not start it (for debug purposes) - generated `generated_core.launch` can be visually inspected for bugs. Using `--debug` tag also triggers `rosrun roslaunch roslaunch-check` on the generated launch file and messages to the user if any failures were found. 

 - `python3 src/magni_robot/magni_bringup/scripts/launch_core.py --launch_generate_path <PATH>` will generate the core.launch at `<PATH>` and launch it from there

- `rosrun magni_bringup launch_core.py -h` Can be run to see all available parameters


## Design choices:
 - we use `systemctl` to start on boot because:
   - can easily be controlled from terminal: `systemctl start/stop/restart magni-base`
   - easy(ish) to debug: `journalctl -u magni-base.service`
   - is a standard way to start stuff on boot
  
 - why robot.yaml is not a rosparam file: 
    - Explicit "overriding" behavior, which is easy to get subtly wrong with rosparam
    - Clearer structure and names that we can set, instead of having to use the rosparam paths
    - Part of the vision was to support non-rosparam-able configuration settings, like what app to launch by default
    - Not everything in there is actually a ROS parameter, like force_time_sync or the camera/lidar extrinsics which is passed directly to xacro
    - Allow for string comparison if statements for determining which nodes to launch, ex: "pi_sonar_v1" vs only having bools

 - why does `launch_core.py` generate `core.launch` and then launches it:
    - Launch_core.py to generate the roslaunch XML based on the parameters. That might reduce the number of places we have to touch to add a new parameter. 
    - Allows for switching between different nodes at launch easier, for things like picking which lidar we have etc.
    - Debugging is much easier: you can actually see the parameters that were used by opening the generated launch file in the `launch` directory of `magni_bringup` package. The alternative is that you leave the param calculation to statements in urdf files which are much harder to debug.
    - Creating the core.launch from python gives much more freedom to do stuff programmatically now and for possible changes in the future

 - ~~internal base.yaml is still kept for non-user editable "parameters".~~ As per discussions [here](https://github.com/UbiquityRobotics/magni_robot/pull/177#issuecomment-993958770) we've moved ALL parameters from `base.yaml` to `robot.yaml` so we don't have confusing situations where people don't know which yaml the parameters are actually being loaded from. `base.yaml` file is kept with a message for pre-noetic users to know where the parameters are aggregated in the noetic architecture.

 - extrinsics of camera and lidar are passed to magni.urdf.xacro from one of the following directories by priority a)`/.ros/extrinsics/`, b)`magni_description/extrinsics/`. This is done by passing the path of the yaml file into the urdf file where the x,y,z,roll,pitch,yaw are extracted from it. This is done because 
   - a) passing down the extrinsics params is in the same format both for launching on real robot (robot.yaml) and for launching in Gazebo (param in empty_world.launch)
   - b) extrinsics in such format can easily be modified by non-expert users (no touching the code or URDF files)
   - c) extrinsics in such format can easily be modified programmatically eg. by auto-extrinsics-calibration programs

 - We decided to move many motor controller and other key operating mode parameters from `base.yaml` to `robot.yaml`. A lot of work was done to do this by Rohan but the issue has been only creeping along and needed closure so that it is now safe to use magni_robot repository along with ubiquity_motor and our main codeline is more supportable again.