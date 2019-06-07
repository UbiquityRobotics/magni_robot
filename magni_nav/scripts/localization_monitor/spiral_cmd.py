#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math

step_size = 1.0
spiral_diam = 7

# linear speed params
a_x = 1.17
v_x = 0.2
a_x_t = v_x/a_x
a_dist = a_x * a_x_t**2 / 2
# angular speed params
a_z = 2.02
v_z = 0.45
a_z_t = v_z/a_z
a_rad = a_z * a_z_t**2 / 2

twist = Twist()


def travel_time(dist):
    t = (dist - a_dist) / v_x + a_x_t
    return t


def rotation_time(rad):
    t = (rad - a_rad) / v_z + a_z_t
    return t


def publish_cmd_vel(twist, t):
    rospy.loginfo("publish cmd:\n" + str(twist))
    end_time = rospy.rostime.get_rostime() + rospy.Duration(t)
    rate = rospy.Rate(100)
    while True:
        pub.publish(twist)
        rate.sleep()
        rem_time = end_time - rospy.rostime.get_rostime()
        if rem_time < rate.sleep_dur:
            if rem_time.to_sec() > 0:
                rospy.sleep(rem_time)
            break


def move(dist):
    twist.linear.x = v_x
    twist.angular.z = 0
    t = travel_time(dist)
    rospy.loginfo("move " + str(dist) + " m for " + str(t) + " sec")
    publish_cmd_vel(twist, t)


def rotate(rad):
    twist.linear.x = 0
    twist.angular.z = v_z
    t = rotation_time(rad)
    rospy.loginfo("rotate " + str(rad) + " rad for " + str(t) + " sec")
    publish_cmd_vel(twist, t)


def stop():
    twist.linear.x = 0
    twist.angular.z = 0
    rospy.loginfo("stop");
    publish_cmd_vel(twist, 0.3)


if __name__ == '__main__':

    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('spiral_cmd', anonymous=True)

    try:
        for i in range(1, spiral_diam):
            dist = i * step_size
            if i < spiral_diam:
                move(dist)
                stop()
                rotate(math.pi/2)
                stop()
                move(dist)
                stop()
                rotate(math.pi/2)
                stop()
            else:
                move(dist)
                stop()
    except rospy.ROSInterruptException:
        pass

