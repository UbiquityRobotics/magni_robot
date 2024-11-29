# ROS2_humble
various nodes as converted to colcon build

for the magni_description, I copied it into a ros2_ws/src, edited CMake.txt and package.xml, and then 
built with colcon_build.

I used:

ros2 launch urdf_tutorial display.launch.py model:=/home/ubiqlap/magni_ws/src/magni_description/urdf/magni.urdf.xacro

to view the model

It is unknow if this will work with magni_bringup

So far I have only tried the plain magni, I haven't tried the tower or loaded the extrinsics

Note there is no /launch file yet. the description.launch ROS1 file needs to be completely rewritten as

description.launch.py for it to work in ROS2.
