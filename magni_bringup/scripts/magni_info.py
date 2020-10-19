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
import rospy
from sensor_msgs.msg import Range
class UbiquitySensors(object):
    def __init__(self):
        self.num_sonars = 5
        self.sonar_ranges = [None] * self.num_sonars

    def rangeCallback(self, msg):
        """Callback for sonars data."""
        words = msg.header.frame_id.split('_')
        idx = int(words[1])

        # save the most recent sonar range
        self.sonar_ranges[idx] = msg.range


def topics_to_file():
    """Run topics and forward output to a file"""
    if not os.path.exists('diagTopic.txt'):
        os.system('touch diagTopic.txt')
    os.system('rostopic echo -n 6 /diagnostics > diagTopic.txt')


if __name__ == "__main__":

    # Check user privileges
    if os.geteuid() != 0:
        sys.exit('Run script as root!')

    rospy.init_node('magni_info')

    print("\nMagni Robot System Information:")
    os.system('date')

    # Status variables
    periodicStatus = False
    verboseOutput = False
    loop_hz = 0.33
    rate = rospy.Rate(loop_hz)
    topics_to_file()

    argcount = len(sys.argv)
    if argcount > 2:
        print("Only -h for help or -p for periodic are allowed as arguments!")
        sys.exit()

    # Possible options
    if argcount == 2:
        if sys.argv[1] == '-h' or sys.argv[1] == '--help':
            print("Inspect key system parameters and exit by default")
            print("use -p for remaining active and monitoring some system parameters")
            sys.exit()
        elif sys.argv[1] == '-v' or sys.argv[1] == '--verbose':
            # set level of verbosity higher
            verboseOutput = True
        elif sys.argv[1] == '-p' or sys.argv[1] == '--periodic':
            # set periodicStatus to True for remaining in status update mode
            periodicStatus = True
        else:
            print("Unrecognized option! We only support -h or -p")
            sys.exit()

    print("\nLinux OS:       --------------------------------")
    os.system('uname -a')
    print("\nHost Information:  --------------------------------")
    os.system('cat /sys/firmware/devicetree/base/model')
    os.system('hostname')
    os.system('hostname -I')
    # TODO: Architecture with hostnamectl
    # TODO: print("\nRoscore: -----------------------------------")
    # TODO: os.system()
    print("\nROS Environmental variables: ----------------")
    os.system('printenv | grep ROS')
    print("Firmware information: -------------------------------")
    os.system('grep -A 1 "Firmware Version" diagTopic.txt | head -2')
    os.system('grep -A 1 "Firmware Date" diagTopic.txt | head -2')
    # TODO: System image version
    print("Magni-base status: ----------------------------------")
    os.system('sudo systemctl status magni-base')
    print("\nDetected I2C devices: -------------------------------")
    # Stop the motor node
    os.system('sudo systemctl stop magni-base.service')
    os.system('sudo i2cdetect -y 1')
    # Restart magni-base service
    os.system('sudo systemctl start magni-base.service')
    # TODO: Check output with subprocess.check_output
    print("\nKey ROS Nodes: ----------------------------")
    print("/motor_node: ")
    out = subprocess.check_output(
        ['rostopic list', '|', 'grep -w "/motor_node"']).decode('utf-8')
    out_info = 'running' if out else 'failed to run'
    print(out_info)
    print("/pi_sonar: ")
    os.system('rostopic list | grep -w "/pi_sonar"')
    print("\nKey ROS Topics: -----------------------------")
    print("/cmd_vel: ")
    os.system('rostopic list | grep -w "/cmd_vel"')
    print("/battery_state: ")
    os.system('rostopic list | grep -w "/battery_state"')
    print("\nROS Log Dir:    --------------------------------")
    os.system('roslaunch-logs')
    print("\npifi Connectivity ------------------------------")
    os.system('pifi --version')
    os.system('pifi status')
    os.system('pifi list seen')
    os.system('pifi list pending')
    print("\nKey Device Info:  --------------------------------")
    os.system('ls -d /dev/ttyAMA0')
    os.system('ls -d /dev/ttyUSB*')
    os.system('ls -d /dev/video0')
    os.system('ls -d /dev/rtc0')
    os.system('ls -d /dev/pigpio')
    print("\nRobot Config:   --------------------------------")
    os.system('cat /etc/ubiquity/robot.yaml')

    # Verbose option
    if verboseOutput:
        print("\nDisk usage:       --------------------------------")
        os.system('df /')
        print("\nMemory Info:      --------------------------------")
        os.system('free | head -2')
        print("\nProcesses:        --------------------------------")
        os.system('ps -ef')
        print("\nDevice Info:      --------------------------------")
        os.system('ls -d /dev/*')
        print("\nBase Config:   ---------------------------------")
        os.system('cat /opt/ros/kinetic/share/magni_bringup/param/base.yaml')
        print("\nNetwork ifconfig:  -----------------------------")
        os.system('ifconfig')
        print("\nNetwork iwconfig:  -----------------------------")
        os.system('iwconfig')
        print("\n/etc/hosts file: -------------------------------")
        os.system('cat /etc/hosts')
        print("\nROS Nodes:   ----------------------------------")
        os.system('rosnode list')
        print("\nROS Topics:  ----------------------------------")
        os.system('rostopic list')
        print("\n.bashrc file ----------------------------------")
        os.system('cat ~/.bashrc')

    # Periodic option
    if periodicStatus:
        us = UbiquitySensors()
        rospy.Subscriber("/sonars", Range, us.rangeCallback)
        print("Periodic monitoring of the robot has been requested. Use Ctrl-C to exit!")

        # TODO: Check raspistill!
        # While our node is running
        while not rospy.is_shutdown():
            print(
                "\n-------------------------------------------------------------------------------------")
            print("Cpu and Memory Stats: ---------------------------------------")
            print('Cpu percent: % ', psutil.cpu_percent())
            tot_m, used_m, free_m = map(int, os.popen(
                'free -t -m').readlines()[-1].split()[1:])
            print('Memory (in KBytes): Total %s  Free %s   Used %s ' %
                  (tot_m, free_m, used_m))
            print("Sonar ranges: ----------------------------------------------")
            print(us.sonar_ranges)
            print("Other System Stats:   ---------------------------------------")
            os.system(
                'rostopic echo -n 4 /diagnostics | grep -A 1 "Battery Voltage"')
            # TODO: Check other informations

            rate.sleep()

    os.system('rm -f diagTopic.txt')
    print("Script Done!")
