#!/usr/bin/env python
import viconClient
import rospy
from geometry_msgs.msg import TransformStamped
def talker():
	rospy.init_node('vicon', anonymous=True)
	rate = rospy.Rate(10)
	try:
		client = viconClient.viconClient("zero", "192.168.1.26")	
		client.connect()
	except Exception as e:
		print "Connection failed: " + str(e)
		return

	while not rospy.is_shutdown():
		rate.sleep()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
