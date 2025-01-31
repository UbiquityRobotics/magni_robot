#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math

# Spiral parameters
STEP_SIZE = 1.0
SPIRAL_DIAM = 7

# Linear speed parameters
A_X = 1.17
V_X = 0.2  # Max linear speed
A_X_T = V_X / A_X
A_DIST = A_X * A_X_T**2 / 2  # Distance needed to reach max speed

# Angular speed parameters
A_Z = 2.02
V_Z = 0.45  # Max angular speed
A_Z_T = V_Z / A_Z
A_RAD = A_Z * A_Z_T**2 / 2  # Rotation radius needed to reach max speed

# ROS Publisher
pub = None

def travel_time(distance):
    """Calculate time needed to travel a given distance."""
    return max((distance - A_DIST) / V_X + A_X_T, 0)

def rotation_time(angle):
    """Calculate time needed to rotate a given angle."""
    return max((angle - A_RAD) / V_Z + A_Z_T, 0)

def move(distance):
    """Move forward by a given distance."""
    twist = Twist()
    twist.linear.x = V_X
    pub.publish(twist)
    rospy.sleep(travel_time(distance))
    stop()

def rotate(angle):
    """Rotate in place by a given angle."""
    twist = Twist()
    twist.angular.z = V_Z
    pub.publish(twist)
    rospy.sleep(rotation_time(angle))
    stop()

def stop():
    """Stop the robot smoothly."""
    pub.publish(Twist())  # Publish zero velocity
    rospy.sleep(0.1)  # Short delay for stability

def shutdown():
    """Ensure the robot stops when the node is shut down."""
    rospy.loginfo("Shutting down, stopping the robot.")
    stop()

if __name__ == '__main__':
    rospy.init_node('spiral_cmd', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.on_shutdown(shutdown)  # Ensure robot stops on exit

    try:
        for i in range(1, SPIRAL_DIAM):
            distance = i * STEP_SIZE
            move(distance)
            if i < SPIRAL_DIAM:
                rotate(math.pi / 2)
    except rospy.ROSInterruptException:
        pass
