#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_localization.srv import *

rospy.init_node('set_init_pose')

rospy.wait_for_service("/set_pose", timeout=None)
set_pose = rospy.ServiceProxy('/set_pose', SetPose)

fid_pose = rospy.wait_for_message('/fiducial_pose', PoseWithCovarianceStamped)
print(fid_pose)

pose_call = SetPoseRequest(fid_pose)
set_pose(pose_call)

