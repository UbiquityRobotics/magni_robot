# Magni bringup

History:
We decided to move many motor controller and other key operating mode parameters from base.yaml to robot.yaml.
A lot of work was done to do this by Rohan but the issue has been only creaping along and needs closure so that it will finally be safe to use magni_robot repository along with ubiquity_motor and our main codeline will then be more supportable again.

**Robot startup procedure:**
Magni robots start ros core and a set of base nodes on boot. This is done in the following order:
1.) using `systemctl` the magni-base service file (`/usr/sbin/magni-base.service`) is ran. This:
  a) starts roscore,
  b) sets up environment variables,
  c) ..., TODO
  d) launches `base.launch`
2.) `base.launch` launches robot base nodes:
  a) `logitech.launch`
  b) `rosbridge.launch` 
  c) placeholder for other nodes that are NOT VARIABLE (don't change with robot config)
  d) starts `launch_core.py`
3.) `launch_core.py` creates and launches `core.launch` (by default in `/tmp/core.launch`) which starts up the rest of the robot nodes that are variable (change with robot config):
  a) motor node: runs ubiquity_motor node with the parameters extracted from either `/etc/ubiquity/robot.yaml` OR `magni_bringup/param/robot.yaml` with respective priorities
  b) robot description: all robot static and dynamic transforms. It gets its camera and lidar extrinsics from either `~/.ros/extrinsics/<SENSOR>_extrinsics_<POSITION>.yaml` OR `magni_description/extrinsics/<SENSOR>_extrinsics_<POSITION>.yaml` with respective priorities. 
  c) other nodes like `controller_spawner`, `diagnostic_aggregator`, `oled_display_node`,... Exactly what gets launched can be seen in `magni_bringup/param/core_launch.em` from which core.launch is generated. Any addition to core.launch should be added inside the `core_launch.em` file

**Useful commands**

`python src/magni_robot/magni_bringup/scripts/launch_core.py --debug` uses launch_core.py to create the core.launch but does not start it (for debug purposes)

`python src/magni_robot/magni_bringup/scripts/launch_core.py --launch_generate_path <PATH>` will generate the core.launch at `<PATH>` and launch it from there





## Design choices (TODO write up nicely)
 - we use `systemctl` to start on boot because:
   - can easily be controlled: `systemctl start/stop/restart magni-base`
   - easy(ish) to debug: `journalctl -u magni-base` TODO correct?
   - is a standard way to start stuff on boot
  
 - why robot.yaml is not a rosparam file: 
    - Explicit "overriding" behavior, which is easy to get subtly wrong with rosparam
    - Clearer structure and names that we can set, instead of having to use the rosparam paths
    - Part of the vision was to support non-rosparam-able configuration settings, like what app to launch by default
    - Not everything in there is actually a ROS parameter, like force_time_sync or the camera/lidar extrinsics which is passed directly to xacro
    - Allow for string comparison if statements for determining which nodes to launch, ex: "pi_sonar_v1" vs only having bools

 - why launch_core.py creates its own xml launch file 
    - launch_core.py to generate the roslaunch XML based on the parameters. That might reduce the number of places we have to touch to add a new parameter.  - It also allows for switching between different nodes at launch easier, for things like picking which lidar we have etc.
    - Debugging is much easier: you can actually see the parameters that were used by opening the generated launch file in the /tmp/ directory. The alternative is that you leave the param calculation to statements in urdf files which are much harder to debug.
    - Creating the core.launch from python gives much more freedom to do stuff programmatically now and for possible changes in the future

 - internal base.yaml is still kept for non-user editable "parameters".

 - extrinsics of camera and lidar are passed to magni.urdf.xacro from one of the following directories by priority a)`/.ros/extrinsics/`, b)`magni_description/extrinsics/`. This is done by passing the path of the yaml file into the urdf file where the x,y,z,roll,pitch,yaw are extracted from it. This is done because a) passing down the extrinsics params is in the same format both for launching on real robot (robot.yaml) and for launching in Gazebo (param in empty_world.launch)