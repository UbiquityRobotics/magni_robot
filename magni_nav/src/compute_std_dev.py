#!/usr/bin/env python

import math
import rospy, sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped


class Handler:
    def __init__(self):
        self.ekg_sub = rospy.Subscriber(
            '/odometry/filtered', Odometry, self.ekf_message_received, queue_size=1)
        self.leica_sub = rospy.Subscriber(
            '/leica', PoseStamped, self.leica_message_received, queue_size=1)
        self.last_ekg_msg = None
        self.last_leica_msg = None
        self.error_sum = 0.0
        self.samples_count = 0
        self.std_dev = 0.0

    def ekf_message_received(self, msg):
        self.last_ekg_msg = msg
        self.compute_std_dev()

    def leica_message_received(self, msg):
        self.last_leica_msg = msg
        self.compute_std_dev()

    def compute_std_dev(self):
        if self.last_ekg_msg is None or self.last_leica_msg is None:
            return

        self.samples_count += 1

        if self.samples_count < 2:
            return

        p1 = self.last_ekg_msg.pose.pose.position
        p2 = self.last_leica_msg.pose.position

        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z
        error = dx * dx + dy * dy + dz * dz

        self.error_sum += error
        self.std_dev = math.sqrt(self.error_sum / (self.samples_count - 1))

        rospy.loginfo("std_dev: [ %.2f ] error: [ %.2f]" % (self.std_dev, error))


if __name__ == '__main__':
    rospy.init_node('compute_std_dev', anonymous=True)
    handler = Handler()
    rospy.spin()
