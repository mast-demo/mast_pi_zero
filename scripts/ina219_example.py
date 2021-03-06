#!/usr/bin/python
import time
import rospy
import math

from math import cos, sin

from geometry_msgs.msg import Vector3, Point, Pose, Quaternion, Twist
from nav_msgs.msg import Odometry

from Subfact_ina219 import INA219_0x40
from Subfact_ina219 import INA219_0x41

ina_r = INA219_0x40()
ina_l = INA219_0x41()

#last_time = 0

def talker():
	pub_r = rospy.Publisher('VI_Sense/right', Vector3, queue_size=10)
	pub_l = rospy.Publisher('VI_Sense/left', Vector3, queue_size=10)
	pub_odom = rospy.Publisher('odom',Odometry, queue_size=10)
	pub_w = rospy.Publisher('w', Vector3, queue_size=10)
	
	x = 0
	y = 0
	theta = 0
	

	rospy.init_node('INA_talker', anonymous=True)
	rate = rospy.Rate(30)#50
	
	while not rospy.is_shutdown():
		#current_time = rospy.Time.now()
		#dt = (current_time - last_time).to_sec()

		#result = ina_r.getBusVoltage_V()
		r_shuntV = ina_r.getShuntVoltage_mV()
		r_busV = ina_r.getBusVoltage_V()
		r_current = ina_r.getCurrent_mA()
		r_loadV = r_busV + (r_shuntV/1000)

		msg = Vector3(r_loadV, r_busV, r_current)
		pub_r.publish(msg)

		#result_l = ina_l.getBusVoltage_V()
		l_shuntV = ina_l.getShuntVoltage_mV()
		l_busV = ina_l.getBusVoltage_V()
		l_current = ina_l.getCurrent_mA()
		l_loadV = l_busV + (l_shuntV/1000)
		
		msg2 = Vector3(l_loadV, l_busV, l_current)
		pub_l.publish(msg2)
		
		#if l_loadV > 0.1 or r_loadV > 0.1:
		velocity_gain = 1.2 #Assuming the legs map to a certain efficiency of wheels
			
		#Calculate and Publish odometry
		R = 0.01 #10mm radius
		L = 0.1 #100mm length between legs
		kb = 0.011 #V/rad/s electromotive force constant (including the gear train and legs) kb = (V-IR)/215 where V = 3.4581 and I = .3151
		w_r = (r_busV - 3.5*r_current/1000)/kb
		w_l = (l_busV - 3.5*l_current/1000)/kb
		#w_r = 680*(r_busV - 3.5*r_current/1000)/60 #rev/sec 753
		#w_r = w_r*2*3.14159 #rad/s
		#w_l = 680*(l_busV - 3.5*l_current/1000)/60 #rev/sec
		#w_l = w_l*2*3.14159 #rad/s
		
		msg_w = Vector3(w_r,w_l,rospy.get_time())  #w_r = w_l & w_l = w_r (These were flipped)
		pub_w.publish(msg_w)

		if l_busV > 2.7 or r_busV > 2.7:	#0.1 need to find out min voltage to move
			#Should be in robot frame of reference
			dx = velocity_gain*R/2*(w_r+w_l)*cos(theta) #Should this just be average velocity?
			dy = velocity_gain*R/2*(w_r+w_l)*sin(theta) #Do not need to report this to Kalman
			dtheta = velocity_gain*R/L*(w_r-w_l)
		
			D = 0.06 #6cm from center of mass (between mid legs) to front of robot (D_r + D_l)/2
		
			x = x + D*cos(theta)
			y = y + D*sin(theta)
			theta = theta + dtheta/50
		else:
			dx = 0
			dy = 0
			dtheta = 0
			x = x
			y = y
			theta = theta

		#Construct Odom msg
		msg3 = Odometry()
		msg3.header.stamp = rospy.Time.now()
		msg3.header.frame_id = 'odom'
		msg3.child_frame_id = 'base_footprint'
		
		cy = cos(0.5*theta)
		q_x = 0
		q_y = 0
		q_z = 0
		q_w = cy
		msg3.pose.pose = Pose(Point(x,y,0),Quaternion(0,0,0,q_w))
		msg3.pose.covariance = (0.01,0,0,0,0,0,  0,0.01,0,0,0,0,  0,0,0.01,0,0,0, 0,0,0,99999,0,0, 0,0,0,0,99999,0, 0,0,0,0,0,0.7 ) 
		msg3.twist.twist = Twist(Vector3(dx,dy,0),Vector3(0,0,dtheta))
		msg3.twist.covariance = (0.01,0,0,0,0,0,  0,0.01,0,0,0,0,  0,0,0.01,0,0,0, 0,0,0,99999,0,0, 0,0,0,0,99999,0, 0,0,0,0,0,0.07 ) 
		pub_odom.publish(msg3)
		#last_time = current_time

		rate.sleep()

try:
	talker()
except rospy.ROSInterruptException:
	print "Failed"
	pass

#while True:
#	result = ina.getBusVoltage_V()
#
#	print "Shunt   : %.3f mV" % ina.getShuntVoltage_mV()
#	print "Bus     : %.3f V" % ina.getBusVoltage_V()
#	print "Current : %.3f mA" % ina.getCurrent_mA()
#	time.sleep(1)
