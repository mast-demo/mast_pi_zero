from json import dumps, loads
from ws4py.client.threadedclient import WebSocketClient
from geometry_msgs.msg import TransformStamped
import time
import os
import rospy

class objectview(object):
    def __init__(self, d):
        self.__dict__ = d

class viconClient(WebSocketClient):
    "Class to create and manage a websocket connection to a rosbridge"
    "server running  publishing vicon topics"
    "arguments are: "
    "   name = name of rover.  must match root of topic namespace"
    "   websocket = websocket address over which to communicate "
    " example:  roverClient(roadrunner1, roadrunner1.local) "
    def __init__(self, name, websocket):
        self.name = name
        self.websocket = websocket
        super(viconClient, self).__init__('ws://' + websocket + ':9090/')
        self.poseTopic = '/vicon/Kamigami_Alpha/Kamigami_Alpha'
        print "Connecting to " + websocket
        self.pub = rospy.Publisher('vicon', TransformStamped, queue_size=5)
        self.skipFrames = 0

    def subscribe(self):
#         self.send('raw\r\n\r\n')
        print "Subscribing"
        msg = {'op': 'subscribe', 'topic': '%s' % self.poseTopic}
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
            if d['topic'] == self.poseTopic:
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
#              print translation.x
              t = TransformStamped()
              t.header.seq = header.seq
              t.header.frame_id = header.frame_id
              t.header.stamp.secs = stamp.secs
              t.header.stamp.nsecs =stamp.nsecs
              t.child_frame_id = child_frame_id
              t.transform.translation.x = translation.x
              t.transform.translation.y = translation.y
              t.transform.translation.z = translation.z
              t.transform.rotation.x = rotation.x
              t.transform.rotation.y = rotation.y
              t.transform.rotation.z = rotation.z
              t.transform.rotation.w = rotation.w
              self.pub.publish(t)

            else:
              print "unknown message on topic " + d['topic']

