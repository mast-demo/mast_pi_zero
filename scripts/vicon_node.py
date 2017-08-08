#!/usr/bin/env python
# subscribe to a vicon transform topic over a rosbridge websocket and 
# publish the transform as a pose message

import viconClient
import rospy
from geometry_msgs.msg import TransformStamped
def talker():
	rospy.init_node('vicon', anonymous=True)
	rate = rospy.Rate(10)
	websocket = rospy.get_param('~websocket','192.168.1.26')
	viconTopic = rospy.get_param('~vicon_topic','vicon')
	poseTopic = rospy.get_param('~pose_topic','pose')
	viconTopic = rospy.get_param('~vicon_topic','192.168.1.26')
	name = rospy.get_param('~name', 'zero')
	try:
		client = viconClient.viconClient(name, websocket, viconTopic, poseTopic)	
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
