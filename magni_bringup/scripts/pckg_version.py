#! /usr/bin/python
import os
import rospkg
import subprocess

rospack = rospkg.RosPack()
rosstack = rospkg.RosStack()
pkgs = ["move_basic", 
	"ubiquity_motor",
	"fiducials",
	"magni_robot",
	"fiducial_slam",
	"raspicam_node",
	"dnn_detect"]

print("\n-- UBIQUITY ROBOTICS PACKAGES VERSION --\n\
----------------------------------------   ")
for pkg in pkgs:
	try:
		os.chdir(rospack.get_path(pkg))
	except:
		os.chdir(rosstack.get_path(pkg))
	finally:
		#  Checks if we are in a Git repository
		if (subprocess.call(["git", "branch"], stderr=subprocess.STDOUT, stdout=open(os.devnull, 'w')) == 0):
			proc = subprocess.Popen(["git", "describe", "--tags"], stdout=subprocess.PIPE, stderr=open(os.devnull, 'w'))
		else: 
			proc = subprocess.Popen(["rosversion", pkg], stdout=subprocess.PIPE)
		# TODO: add option if ros stack is not git repo - read it from the xml.file
		version = proc.communicate()
		print("(%s) - %s" % (pkg, version[0].decode('utf-8')))
