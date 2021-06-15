# Magni bringup

History:
We decided to move many motor controller and other key operating mode parameters from base.yaml to robot.yaml.
A lot of work was done to do this by Rohan but the issue has been only creaping along and needs closure so that it will finally be safe to use magni_robot repository along with ubiquity_motor and our main codeline will then be more supportable again.

## Design choices (TODO write up nicely)
 - why robot.yaml is not a rosparam file: 
    - Explicit "overriding" behavior, which is easy to get subtly wrong with rosparam
    - Clearer structure and names that we can set, instead of having to use the rosparam paths
    - Part of the vision was to support non-rosparam-able configuration settings, like what app to launch by default
    - Not everything in there is actually a ROS parameter right now, like force_time_sync or the camera position which is passed directly to xacro
    - Allow for string comparison if statements for determining which nodes to launch, ex: "pi_sonar_v1" vs only having bools

 - why launch_core.py creates its own xml lanch file 
   - launch_core.py to generate the roslaunch XML based on the parameters. That might reduce the number of places we have to touch to add a new parameter. It also allows for switching between different nodes at launch easier, for things like picking which lidar we have etc.
   - debugging is much easier: you can actually see the parameters that were used by opening the generated launch file in the /tmp/ directory. The alternative is that you leave the param calculation to statemets in urdf files which are much harder to debug.


 - still keep the internal base.yaml for non-user editable "parameters".

 - extrinsiscs of camera and lidar are passed to magni.urdf.xacro from one of the following directories by priority a) /.ros/extrinsics/, b)magni_description/param. This is done by passing the path of the yaml file into the urdf file where the x,y,z,roll,pitch,yaw are extracted from the yaml file. This is done because a) passing down the extrinsics params is in the same format both for launching on real robot and for gazebo - the filesystem is simpler and there is less files to maintain. The only difference in controlling which extrinsics yaml file is going to be used between launching in Gazebo and on real robot is that: In gazebo you need to set that in magni_gazebo empty_world.launch and on real robot in the /etc/ubiquity/robot.yaml. From both files you are just setting the string of the file and not the actuall extrinsics parameters themselves 