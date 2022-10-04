#!/usr/bin/python3

"""
Copyright (c) 2018, Ubiquity Robotics
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of fiducial_follow nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

"""
Send a goal to move_basic to move a specified amount
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
import move_base_msgs.msg
import tf.transformations
import tf2_ros
import tf2_geometry_msgs
import math
import traceback
from geometry_msgs.msg import Quaternion, Point, PoseStamped

# Utility function to convert radians to degrees
def degrees(r):
    return 180.0 * r / math.pi

# Utility function to convert degrees to radians
def radians(r):
    return math.pi * r / 180.0

# Utility function to return an identiy quaternion representing zero rotation
def identity_quaternion():
    return Quaternion(0,0,0,1)

class Move:
    """
    Constructor for our class
    """
    def __init__(self):
        # Set up transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create a proxy object for the move action server
        self.move = actionlib.SimpleActionClient('/move_base',
                                                 move_base_msgs.msg.MoveBaseAction)
        rospy.loginfo("Waiting for move service to be available")
        if not self.move.wait_for_server(rospy.Duration(10.0)):
           rospy.logerr("Move service not available")
           self.move = None
           return

    # Go to a goal
    def goto_goal(self, orientation=identity_quaternion(), position=Point()):
        if self.move is None:
            rospy.logerr("Move service not available")
            return False
        # Transform goal into the odom fram
        # Create goal and actionlib call to rotate
        pose_base = PoseStamped()
        pose_base.header.frame_id = "base_link"
        pose_base.pose.orientation = orientation
        pose_base.pose.position = position

        try:
            pose_odom = self.tf_buffer.transform(pose_base, "odom",
                                                 rospy.Duration(1.0))
        except:
            rospy.logerr("Unable to transform goal into odom frame")
            return
        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose = pose_odom
        self.move.send_goal(goal)
        self.move.wait_for_result(rospy.Duration(60.0))

        if self.move.get_state() == GoalStatus.SUCCEEDED:
           rospy.loginfo("Success")
           return True
        else:
           rospy.loginfo("Failure")
           return False

    def rotate(self, angle):
        rospy.loginfo("Rotating %f degrees" % angle)
        q = tf.transformations.quaternion_from_euler(0, 0, radians(angle))
        self.goto_goal(orientation=Quaternion(*q))

    def forward(self, dist):
        rospy.loginfo("Moving %f meters" % dist)
        self.goto_goal(position=Point(dist, 0, 0))

def usage():
    print( \
"""Usage: %s rotate|forward amount
  rotate <angle in degrees>
  forward <distance in meters>
""" % sys.argv[0])
    sys.exit(1)


if __name__ == "__main__":
    rospy.init_node("rotate")
    if len(sys.argv) != 3:
        usage()
    
    if sys.argv[1] == "rotate":
        node = Move()
        node.rotate(float(sys.argv[2]))
    elif sys.argv[1] == "forward":
        node = Move()
        node.forward(float(sys.argv[2]))
    else:
        usage()
