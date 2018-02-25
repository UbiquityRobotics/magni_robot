#!/usr/bin/python

###
# Eventually this should do something smarter, like launch sonars
# 3DTOF, etc, if they are installed.
###

from subprocess import call
call(["roslaunch", "magni_bringup", "core.launch"])