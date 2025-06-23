This is my first attemp at a ROS2 magni_description node.

Lots of unknowns, especially passing different configurations
selected in magni_bringup. It is set-up as a stand alone node.
It is setup as plain vanilla magni with camera in forward 
position and lidar at top plate  center.

A rviz configuration file has been added to /config.

ros2 run joint_state_publisher_gui joint_state_publisher_gui
ros2 run rviz2 rvzi2

seems to work, but needs tuning as two instances of jsp are launched.
