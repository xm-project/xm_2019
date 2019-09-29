#!/usr/bin/env python
# encoding:utf-8
import subprocess
import rospy
a = subprocess.Popen('xterm -e rosrun xm_vision people_tracking ',shell = True)


print a.returncode
if a.returncode == None:
	print 'fuck'
