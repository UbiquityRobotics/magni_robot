#!/usr/bin/env python

import rospy
from raspicam_node.msg import MotionVectors
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from skvideo.motion import blockMotion
import numpy as np

MACROBLOCK_SIZE = 16


class MotionVectorPublisher:
    """Motion vector publisher that uses the Diamond Search algorithm (Zhu and Ma, 2000)
    to estimate the motion. Can be used in gazebo simulation as an alternative for
    the Raspberry Pi built-in motion estimation.
    """
    def __init__(self, subscribe_topic, publish_topic):
        self.previous_image = None
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(
            publish_topic, MotionVectors, queue_size=10)
        self.subscriber = rospy.Subscriber(
            subscribe_topic, Image, self.image_callback)

    def _compute_sad(self, b1, b2):
        return np.sum(np.abs(b1 - b2))

    def image_callback(self, img):
        try:
            image = self.bridge.imgmsg_to_cv2(img, 'rgb8')
        except CvBridgeError:
            rospy.logerr("Failed to convert the image.")
            return
        if self.previous_image is None:
            self.previous_image = image
            return

        mbs = MACROBLOCK_SIZE
        video = np.vstack([self.previous_image.reshape(-1, *self.previous_image.shape),
                           image.reshape(-1, *image.shape)])
        motion = blockMotion(video, method='DS', mbSize=mbs, p=2)[0]

        sad_vals = np.zeros((motion.shape[0], motion.shape[1]), dtype=np.uint16)
        for i in range(0, image.shape[0] - mbs + 1, mbs):
            for j in range(0, image.shape[1] - mbs + 1, mbs):
                new_block_i = i + motion[np.int(i / mbs), np.int(j / mbs), 1]
                new_block_j = j + motion[np.int(i / mbs), np.int(j / mbs), 0]
                sad = self._compute_sad(
                    self.previous_image[i: i + mbs, j: j + mbs].astype(np.float32),
                    image[new_block_i: new_block_i + mbs, new_block_j: new_block_j + mbs].astype(np.float32))
                sad_vals[np.int(i / mbs), np.int(j / mbs)] = sad

        message = MotionVectors(
            mbx=motion.shape[0], mby=motion.shape[1],
            x=motion[:, :, 0].reshape(-1), y=motion[:, :, 1].reshape(-1),
            sad=sad_vals.reshape(-1))

        self.publisher.publish(message)
        self.previous_image = image


if __name__ == '__main__':
    try:
        rospy.init_node('motion_vector_publisher')
        subscribe_topic = rospy.get_param('~subscribe_topic')
        publish_topic = rospy.get_param('~publish_topic')
        MotionVectorPublisher(subscribe_topic, publish_topic)
        rospy.spin()
    except rospy.ROSInterruptException:
        exit(1)
