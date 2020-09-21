#!/usr/bin/env python

import rospy
from raspicam_node.msg import MotionVectors


class VisualVectorSubscriber:

    def __init__(self, subscribe_topic):
        self.subscriber = rospy.Subscriber(
            subscribe_topic, MotionVectors, self.callback)

    def callback(self, data):
        # TODO fill this method with desired code
        # `data` variable contains visual vectors of the following form:
        #
        # std_msgs/Header header - Message header
        #
        # uint8 mbx - Number of macroblocks in horizontal dimension
        #
        # uint8 mby - Number of macroblocks in vertical dimension
        #
        # int8[] x - Horizontal component of visual vectors
        #
        # int8[] y - Vertical component of visual vectors
        #
        # uint16[] sad -Sum of Absolute Difference metric of visual vectors
        pass


if __name__ == '__main__':
    try:
        rospy.init_node('visual_vector_subscriber')
        VisualVectorSubscriber('/raspicam_node/motion_vectors')
        rospy.spin()
    except rospy.ROSInterruptException:
        exit(1)
