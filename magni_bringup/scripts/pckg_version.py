#! /usr/bin/python

import subprocess

pkgs = ["move_basic", 
	"fiducials",
	"ubiquity_motor",
	"magni_robot",
	"raspicam_node",
	"dnn_detect"]

print("\n-- UBIQUITY ROBOTICS PACKAGES VERSION --")
for pkg in pkgs:
	proc = subprocess.Popen(["rosversion", pkg], stdout=subprocess.PIPE)
	version = proc.communicate()
	print("%s - %s" % (pkg, version[0].decode('utf-8')))
	# TODO: Git hash
