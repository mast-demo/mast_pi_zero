#!/usr/bin/env python
# 
# subscribes to cmd_vel messages and 
# transfers that to PWM driver for motor 4tronix motor Shim
# linear x is interpreted as % effort forward and backward
# angular z is turn in place left and right
# angular is ignored if linear is non-zero
# commands expire aftwer watchdogCount seconds
#

import rospy
import pzm
from geometry_msgs.msg import Twist

pzm.init()
watchdogCount = 0

def callback(data):
	#rospy.loginfo("callback: "+ str(data.linear.x))
	global watchdogCount
	watchdogCount = 5
	if data.linear.x > 0.0:
		pzm.forward(data.linear.x)
	elif data.linear.x < 0.0:
		pzm.reverse(-data.linear.x)
	elif data.angular.z > 0:
		pzm.spinLeft(data.angular.z)
	elif data.angular.z < 0:
		pzm.spinRight(-data.angular.z)

def timerCallback(event):
	global watchdogCount
	if watchdogCount <= 0:
		pzm.stop()
	else:
		watchdogCount = watchdogCount - 1

def listener():
	rospy.init_node('motor', anonymous=True)
	rospy.Subscriber("cmd_vel", Twist, callback)
	rospy.Timer(rospy.Duration(1), timerCallback)
	rospy.spin()

if __name__ == '__main__':
	listener() 
