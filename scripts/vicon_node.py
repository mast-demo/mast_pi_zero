#!/usr/bin/env python
# subscribe to a vicon transform topic over a rosbridge websocket and 
# publish the transform as a pose message

import viconClient
import rospy
from geometry_msgs.msg import TransformStamped
def talker():
	rospy.init_node('vicon', anonymous=True)
	rate = rospy.Rate(10)
	websocket = rospy.get_param('~websocket','192.168.1.6')
	name = rospy.get_param('~name', 'zero')
	viconTopic = rospy.get_param('~vicon_topic','/vicon/'+name+'/'+name)
	poseTopic = rospy.get_param('~pose_topic','pose')
	try:
		client = viconClient.viconClient(name, websocket, viconTopic, poseTopic)	
		client.connect()
		print "Connected"
	except Exception as e:
		print "Connection failed: " + str(e)
		return

	rospy.spin()

if __name__ == '__main__':
  try:
    talker()
  except rospy.ROSInterruptException:
    pass
