from json import dumps, loads
from ws4py.client.threadedclient import WebSocketClient
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import os
import rospy

class objectview(object):
    def __init__(self, d):
        self.__dict__ = d

class viconClient(WebSocketClient):
    "Class to create and manage a websocket connection to a rosbridge"
    "server publishing vicon transform topics"
    "arguments are: "
    "   name = name of rover.  must match root of topic namespace"
    "   websocket = websocket address over which to communicate "
    "   vicon_topic = name of vicon topic to subscribe to "
    "   pose_topic = name of topic on which to publish the pose "
    " example:  roverClient('pi', '192.168.1.1', '/vicon/Kamagami_Alpha/Kamagami_Alpha) "
    def __init__(self, name, websocket, vicon_topic, pose_topic):
        self.name = name
        self.websocket = websocket
        super(viconClient, self).__init__('ws://' + websocket + ':9090/')
        self.viconTopic = vicon_topic
        print "Vicon node named "+name+" Connecting to topic " +vicon_topic + " socket " + websocket
        self.pub = rospy.Publisher(pose_topic, PoseWithCovarianceStamped, queue_size=5)
        self.skipFrames = 0

    def subscribe(self):
#         self.send('raw\r\n\r\n')
        print "Subscribing"
        msg = {'op': 'subscribe', 'topic': '%s' % self.viconTopic}
        self.send(dumps(msg))

    def opened(self):
        print "Connection opened..."
        self.subscribe()

    def closed(self, code, reason=None):
        print code, reason

    def received_message(self, m):
        if self.skipFrames < 10:
          self.skipFrames = self.skipFrames + 1
          return
        self.skipFrames = 0
        d=loads(str(m))
#        print d['op']
        if d['op'] == 'service_response':
            print 'Service Response'
        elif d['op'] == 'publish':
            if d['topic'] == self.viconTopic:
#              print d['msg']
              msg = objectview(d['msg'])
#              print msg.header
#              print msg.transform
              header = objectview(msg.header)
              stamp = objectview(header.stamp)
              child_frame_id = msg.child_frame_id
              transform = objectview(msg.transform)
              translation = objectview(transform.translation)
              rotation = objectview(transform.rotation)
# publish the transform as a pose
              p = PoseWithCovarianceStamped()
              p.header.seq = header.seq
              p.header.frame_id = header.frame_id
              p.header.stamp.secs = stamp.secs
              p.header.stamp.nsecs = stamp.nsecs
              p.pose.pose.position.x = translation.x
              p.pose.pose.position.y = translation.y
              p.pose.pose.position.z = translation.z
              p.pose.pose.orientation.x = rotation.x
              p.pose.pose.orientation.y = rotation.y
              p.pose.pose.orientation.z = rotation.z
              p.pose.pose.orientation.w = rotation.w
              self.pub.publish(p)

            else:
              print "unknown message on topic " + d['topic']

