To do simple moves:

rosrun magni_nav simple_move.py forward 1  # moves robot 1 meter forward,  -1 will turn 180, go back, turn 180

rosrun magni_nav simple_move.py rotate 90   # ritates 90 degrees

by altenateing in a script could draw a square, etc.

timed_out_and_back  = to  forward -1

move_base_square = to script 

if for some reason move_base isn't working corectly, with the robot or magni_gazebo running 
in separate ssh window try:

 rostopic pub -r 1 /cmd_vel geometry_msgs/Twist -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 1.0]'
