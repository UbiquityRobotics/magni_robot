#!/usr/bin/python

###
# Eventually this should do something smarter, like launch sonars
# 3DTOF, etc, if they are installed.
###

import roslaunch
roslaunch.main(argv=["roslaunch", "magni_bringup", "core.launch"])
