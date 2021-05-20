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


 - still keep the internal base.yaml for non-user editable "parameters".
