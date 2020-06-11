#!/usr/bin/python

"""
Copyright (c) 2020, Ubiquity Robotics
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of magni_robot nor the names of its
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
magni_info is meant to gather up a variety of information for the Magni robot platform
A report is then generated with the latest system information available
"""

import os
import sys
import subprocess
import psutil
import roslib
import rospy

from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Range

import tf2_ros
from math import pi, sqrt, atan2
import traceback
import math
import time

# Set the rate the main loop will run at
loop_hz = 0.33

# optionally we can disable verbose debug messages or leave them on as in legacy code
# 1 is less verbose and 2 more verbose with 0 almost no messages
debug_mode  = rospy.get_param("~debug_mode", 1)

num_sonars   = 5
sonar_ranges = [None] * num_sonars

def degrees(r):
    return 180.0 * r / math.pi


"""
Called when a sonar message is received
"""
def rangeCallback( msg):

    #if msg.header.frame_id == "sonar_1" or msg.header.frame_id == "sonar_2" or msg.header.frame_id == "sonar_3":
    words = msg.header.frame_id.split('_')    # we expect frame_id like  sonar_3 for number 3
    idx = int(words[1])

    # save the most recent sonar range
    sonar_ranges[idx] = msg.range
    # print ("Sonar %s range %s" % (idx, msg.range))

    #if msg.header.frame_id == "sonar_3":
    #    print msg.header.frame_id, words[1], idx, sonar_ranges[3], msg.range, 

"""
Main loop
"""
def run():
    print "INIT: Start the run main thread"
    # setup for looping at 25hz
    rate = rospy.Rate(loop_hz)
    secPerLoop = 1.0 / loop_hz

    print "system monitor starting with looprate %d debug %d" %  (loop_hz, debug_mode)

    num_sonars   = 5
    sonar_ranges = [None] * num_sonars

    # ------------------------------------------------------------------
    # Setup ROS topics to be published or subscribed to for operation

    # A publisher for robot motion commands
    cmdvelPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # Subscribe to sonar range sensors for object detection
    rospy.Subscriber("/sonars", Range, rangeCallback)

    # While our node is running
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('magni_info')

    print "\nMagni Robot System Information:"
    os.system('date')

    # setup for looping at 25hz
    rate = rospy.Rate(loop_hz)
    secPerLoop = 1.0 / loop_hz

    # set periodicStatus to non-zero for remaining in status update mode
    periodicStatus = 0
    verboseOutput  = 0

    # We are going to do very simple option checks for help or for periodic mode 
    argcount = len(sys.argv) - 1
    if argcount > 1:
        print("Only -h for help or -p for periodic are allowed as arguments!")
        exit()

    # simple and swift checking for only two possible options
    periodicStatus = 0
    if argcount == 1:
        if sys.argv[1] == '-h' or sys.argv[1] == '--help':
            print("Inspect key system parameters and exit by default")
            print("use -p for remaining active and monitoring some system parameters")
            exit()
        elif sys.argv[1] == '-p' or sys.argv[1] == '--periodic':
            # set periodicStatus to non-zero for remaining in status update mode
            periodicStatus = 1
        elif sys.argv[1] == '-v' or sys.argv[1] == '--verbose':
            # set level of verbosity higher (not for periodic mode)
            verboseOutput = 1
        else:
            print("Unrecognized option! we support -h or -p")
            print("Use -p for remaining active and monitoring some system parameters")
            exit()


    # ------------------------------------------------------------------
    # Setup ROS topics to be published or subscribed to for operation

    # A publisher for robot motion commands
    cmdvelPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    # Subscribe to sonar range sensors for object detection
    rospy.Subscriber("/sonars", Range, rangeCallback)

    # gather system info that will not change
    # out = subprocess.Popen(cmd, 
    #     stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    # stdout,stderr = out.communicate()
    # print(stdout)

    print("\nLinux OS:       --------------------------------")
    cmd = 'uname -a' 
    os.system(cmd)
    print("\nHost Cpu Type:  --------------------------------")
    cmd = 'cat /sys/firmware/devicetree/base/model' 
    os.system(cmd)
    print " "
    print "\nDisk usage:       --------------------------------"
    cmd = 'df /' 
    os.system(cmd)
    print "\nMemory Info:      --------------------------------"
    os.system(cmd)
    cmd = 'free | head -2' 
    os.system(cmd)
    if verboseOutput > 0:
        print "Processes:        --------------------------------"
        cmd = 'ps -ef' 
        os.system(cmd)
    if verboseOutput > 0:
        print "Device Info:      --------------------------------"
        cmd = 'ls -d /dev/*'
    else:
        print "Key Device Info:  --------------------------------"
        cmd = 'ls -d /dev/ttyAMA0'
        os.system(cmd)
        cmd = 'ls -d /dev/ttyUSB*'
        os.system(cmd)
        cmd = 'ls -d /dev/video0'
        os.system(cmd)
        cmd = 'ls -d /dev/rtc0'
        os.system(cmd)
        cmd = 'ls -d /dev/pigpio'
        os.system(cmd)
    print "\nROS Log Dir:    --------------------------------"
    cmd = 'roslaunch-logs'
    os.system(cmd)
    if verboseOutput > 0:
        cmd = 'ls -ldtr `roslaunch-logs`/*.*'
        os.system(cmd)
    print "\nRobot Config:   --------------------------------"
    cmd = 'cat /etc/ubiquity/robot.yaml'
    os.system(cmd)
    if verboseOutput > 0:
        print "\nBase Config:   ---------------------------------"
        cmd = 'cat /opt/ros/kinetic/share/magni_bringup/param/base.yaml'
        os.system(cmd)
    print "\nROS Log Dir:    --------------------------------"
    cmd = 'roslaunch-logs'
    os.system(cmd)
    print "\nHostname And Network:  -------------------------"
    cmd = 'hostname'
    os.system(cmd)
    cmd = 'hostname -I'
    os.system(cmd)
    print "\npifi Connectivity ------------------------------"
    cmd = 'pifi status'
    os.system(cmd)
    cmd = 'pifi list pending'
    os.system(cmd)
    if verboseOutput > 0:
        print "\nNetwork ifconfig:  -----------------------------"
        cmd = 'ifconfig'
        os.system(cmd)
        print "\nNetwork iwconfig:  -----------------------------"
        cmd = 'iwconfig'
        os.system(cmd)
    if verboseOutput > 0:
        print "\nROS Nodes:   ----------------------------------"
        cmd = 'rosnode list'
        os.system(cmd)
        print "\nROS Topics:  ----------------------------------"
        cmd = 'rostopic list'
    else:
        print "\nKey ROS Nodes:  -------------------------------"
        cmd = 'rosnode list | grep motor_node'
        os.system(cmd)
        cmd = 'rosnode list | grep pi_sonar'
        os.system(cmd)
    print "\nDiagnostic Topic Info:  ------------------------"
    cmd = 'rostopic echo -n 6 /diagnostics > diagTopic.txt'
    os.system(cmd)
    cmd = 'grep -A 1 "Firmware Version" diagTopic.txt | head -2'
    os.system(cmd)
    cmd = 'grep -A 1 "Firmware Date" diagTopic.txt | head -2'
    os.system(cmd)
    cmd = 'grep -A 1 "Battery Voltage" diagTopic.txt | head -2'
    os.system(cmd)
    cmd = 'grep -A 1 "Motor Power" diagTopic.txt | head -2'
    os.system(cmd)
    print "Sonar Ranges:         ---------------------------------------"
    print(sonar_ranges)

    if periodicStatus > 0:
        print "Periodic monitoring of the robot has been requested. Use Ctrl C to exit "

        # While our node is running
        while not rospy.is_shutdown():
            print "\n-------------------------------------------------------------------------------------"
            print "Cpu and Memory Stats: ---------------------------------------"
            print('Cpu percent: % ', psutil.cpu_percent())
            tot_m, used_m, free_m = map(int, os.popen('free -t -m').readlines()[-1].split()[1:])
            print('Memory (in KBytes): Total %s  Free %s   Used %s ' % (tot_m, free_m, used_m))
            print "Sonar Ranges:         ---------------------------------------"
            print(sonar_ranges)
            print "Other System Stats:   ---------------------------------------"
            cmd = 'rostopic echo -n 4 /diagnostics | grep -A 1 "Battery Voltage"'
            os.system(cmd)

            rate.sleep()

    print "Script Done"
