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
        rospy.Subscriber("/sonars", Range, self.rangeCallback)

    def rangeCallback(self, msg):
        """Callback for sonars data."""
        words = msg.header.frame_id.split('_')
        idx = int(words[1])

        # save the most recent sonar range
        self.sonar_ranges[idx] = msg.range


def topics_to_file():
    """Run topics and forward output to a file"""
    if not os.path.exists('Topics.txt'):
        os.system('touch Topics.txt')
    if not os.path.exists('Nodes.txt'):
        os.system('touch Nodes.txt')
    if not os.path.exists('diagnostics.txt'):
        os.system('touch diagnostics.txt')
    os.system('rostopic list >> Topics.txt')
    os.system('rosnode list >> Nodes.txt')
    os.system('rostopic echo -n 10 /diagnostics > diagnostics.txt')


if __name__ == "__main__":

    rospy.init_node('magni_info')
    us = UbiquitySensors()
    topics_to_file()
    periodicStatus = False
    verboseOutput = False
    loop_hz = 0.33
    rate = rospy.Rate(loop_hz)

    # User input
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

    print("\nMagni Robot System Information:")
    os.system('date')
    print("\n------------------------- Linux OS: --------------------------------")
    os.system('hostnamectl')
    print("\n------------------------- Host Information:  -----------------------")
    os.system('cat /sys/firmware/devicetree/base/model')
    print("")
    os.system('hostname -I')
    print("\n------------------------- ROS Environmental variables: -------------")
    os.system('printenv | grep ROS')
    print("\n------------------------- Firmware information: --------------------")
    os.system('grep -A 1 "Firmware Version" diagnostics.txt | head -2')
    os.system('grep -A 1 "Firmware Date" diagnostics.txt | head -2')
    os.system('grep -A 1 "Battery Voltage" diagnostics.txt | head -2')
    os.system('grep -A 1 "Motor Power" diagnostics.txt | head -2')
    print("\n------------------------- Detected I2C devices: --------------------")
    # Stop the motor node
    os.system('sudo systemctl stop magni-base.service')
    os.system('sudo i2cdetect -y 1')
    # Restart magni-base service
    os.system('sudo systemctl start magni-base.service')
    print("\n------------------------- Key ROS Nodes: ---------------------------")
    try:
        out = subprocess.check_output(
            ['grep', '-w', '/motor_node', 'Nodes.txt']).decode('utf-8')
        sys.stdout.write(out)
    except subprocess.CalledProcessError:
        print("/motor_node not running!")
    try:
        out = subprocess.check_output(
            ['grep', '-w', '/pi_sonar', 'Nodes.txt']).decode('utf-8')
        sys.stdout.write(out)
    except subprocess.CalledProcessError:
        print("/pi_sonar not running!")
    print("\n------------------------- Key ROS Topics: --------------------------")
    try:
        out = subprocess.check_output(
            ['grep', '-w', '/cmd_vel', 'Topics.txt']).decode('utf-8')
        sys.stdout.write(out)
    except subprocess.CalledProcessError:
        print("/cmd_vel topic not running!")
    try:
        out = subprocess.check_output(
            ['grep', '-w', '/battery_state', 'Topics.txt']).decode('utf-8')
        sys.stdout.write(out)
    except subprocess.CalledProcessError:
        print("/battery_state topic not running!")
    print("\n------------------------ ROS Log Dir: -----------------------------")
    os.system('roslaunch-logs')
    print("\n------------------------ pifi Connectivity ------------------------")
    os.system('pifi --version')
    print("# Status: ")
    os.system('pifi status')
    print("# Seen Wifi's: ")
    os.system('pifi list seen')
    print("# Pending Wifi's: ")
    os.system('pifi list pending')
    print("\n------------------------ Key Device Info: -------------------------")
    os.system('ls -d /dev/ttyAMA0 2>/dev/null')
    os.system('ls -d /dev/ttyUSB* 2>/dev/null')
    os.system('ls -d /dev/video0 2>/dev/null')
    os.system('ls -d /dev/rtc0 2>/dev/null')
    os.system('ls -d /dev/pigpio 2>/dev/null')
    print("\n------------------------ Robot Config: ----------------------------")
    os.system('cat /etc/ubiquity/robot.yaml')

    # Verbose option
    if verboseOutput:
        print("\n-------------------- Disk usage: ------------------------------")
        os.system('df /')
        print("\n-------------------- Memory Info: -----------------------------")
        os.system('free | head -2')
        print("\n-------------------- Processes: -------------------------------")
        os.system('ps -ef')
        print("\n-------------------- Device Info: -----------------------------")
        os.system('ls -d /dev/*')
        print("\n-------------------- Base Config: -----------------------------")
        os.system('cat /opt/ros/kinetic/share/magni_bringup/param/base.yaml')
        print("\n-------------------- Network ifconfig -------------------------")
        os.system('ifconfig')
        print("\n-------------------- Network iwconfig: ------------------------")
        os.system('iwconfig')
        print("\n-------------------- /etc/hosts file: -------------------------")
        os.system('cat /etc/hosts')
        print("\n-------------------- ROS Nodes: -------------------------------")
        os.system('rosnode list')
        print("\n-------------------- ROS Topics: ------------------------------")
        os.system('rostopic list')
        print("\n-------------------- .bashrc file -----------------------------")
        os.system('cat ~/.bashrc')

    # Periodic option
    if periodicStatus:
        print("Periodic monitoring of the robot has been requested. Use Ctrl-C to exit!")

        # While our node is running
        while not rospy.is_shutdown():
            print(
                "\n-------------------------------------------------------------------------------------")
            print("------------------- Cpu and Memory Stats --------------------")
            print('Cpu percent: % ', psutil.cpu_percent())
            tot_m, used_m, free_m = map(int, os.popen(
                'free -t -m').readlines()[-1].split()[1:])
            print('Memory (in KBytes): Total %s  Free %s   Used %s ' %
                  (tot_m, free_m, used_m))
            print("------------------- Sonar ranges: ---------------------------")
            print(us.sonar_ranges)

            rate.sleep()

    # Delete temporary files
    os.system('rm -f diagnostics.txt')
    os.system('rm -f Nodes.txt')
    os.system('rm -f Topics.txt')

    print('##### System information script done! #####')
