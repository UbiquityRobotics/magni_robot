#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped


class Handler:
    def __init__(self):
        self.fiducial_sub = rospy.Subscriber(
            '/fiducial_pose', PoseWithCovarianceStamped, self.fiducial_message_received, queue_size=1)
        self.fiducial_pub = rospy.Publisher(
            '/fiducial_pose/fixed', PoseWithCovarianceStamped, queue_size=1)
        self.odometry_sub = rospy.Subscriber(
            '/odom', Odometry, self.odometry_message_received, queue_size=1)
        self.odometry_pub = rospy.Publisher(
            '/odom/fixed', Odometry, queue_size=1)

    def odometry_message_received(self, msg):
        result = Odometry()
        result.header = msg.header
        result.pose.pose = msg.pose.pose
        result.twist.twist = msg.twist.twist
        self.odometry_pub.publish(result)

    def fiducial_message_received(self, msg):
        result = PoseWithCovarianceStamped()
        result.header = msg.header
        result.pose.pose = msg.pose.pose
        self.fiducial_pub.publish(result)


if __name__ == '__main__':
    rospy.init_node('nullify_covariance', anonymous=True)
    handler = Handler()
    rospy.spin()
