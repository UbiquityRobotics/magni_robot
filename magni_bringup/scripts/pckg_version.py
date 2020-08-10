#! /usr/bin/python
import os
import rospkg
import subprocess

rospack = rospkg.RosPack()
pkgs = ["move_basic", 
	"fiducials",
	"ubiquity_motor",
	"magni_robot",
	"raspicam_node",
	"dnn_detect"]

print("\n-- UBIQUITY ROBOTICS PACKAGES VERSION --")
for pkg in pkgs:
	try:
		os.chdir(rospack.get_path(pkg))
	except:
		print("Package (%s) not found.\n" % pkg)
		continue

	#  Checks if we are in a Git repository
	if (subprocess.call(["git", "branch"], stderr=subprocess.STDOUT, stdout=open(os.devnull, 'w')) == 0):
		proc = subprocess.Popen(["git", "describe", "--tags"], stdout=subprocess.PIPE, stderr=open(os.devnull, 'w'))
	else: 
		proc = subprocess.Popen(["rosversion", pkg], stdout=subprocess.PIPE)

	version = proc.communicate()
	print("(%s) - %s" % (pkg, version[0].decode('utf-8')))
